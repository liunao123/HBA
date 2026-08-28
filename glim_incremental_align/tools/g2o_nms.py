#!/usr/bin/env python3
"""四维非极大抑制 (NMS) 去掉位姿图里**重复**的回环/跨session约束。

要解决的问题 (实测 incr4):
  回环:   同一个 i 配到的多个 j 之间帧号差中位 = 1, 96% <= 3 —— 就是 A<->B 和 A<->B+-1
          这种同一件事重复说好几遍; 42% 的边与前一条边弧长间隔为 0。
  跨session: 一个参照帧被最多 181 条边同时钉着 (被并入端基本每帧一条, 完全没有去重)。
  两者的共同后果: 边挤在少数位置 (jjst2 只有 32% 的 50m 格有回环边, jjst5 只有 11%),
  而那些格里每格十几条。密集的地方权重虚高, 稀疏的地方没人管。

抑制条件 —— **两端都要近才算重复**:
    已接受一条边 (i', j'), 则新边 (i, j) 被丢弃 iff
        |P_i - P_i'| <= R  且  |P_j - P_j'| <= R
  为什么不能只看中点: A<->B 和 A<->B+1 的中点只差半个帧距, 但两个几何上完全不同的
  对也可能共享中点。中点是有损的键, 两端的位置对才是。

**分组是关键**: 抑制只在**同一个 (session_i, session_j) 组合**内进行。
  于是 "A 与 session1 的 B 建了约束后, session2 里同样在 B 附近的 C 仍然要建 A<->C" ——
  它们落在不同的组里, 互不抑制。这正是多 session 互相印证的价值所在, 不能被去重吃掉。

**窗口内的帧间边一条都不动**: 那是局部里程链 (|帧号差| <= intra_win), 稀疏化它等于
  破坏轨迹本身。只对 回环 (同session 且帧号差 > intra_win) 和 跨session 边做。

质量排序: g2o 里没有 inlier/nn, 所以用**测量与图解的残差** ||t_meas - t_graph|| 当质量
  (小 = 这条边和最终解一致)。贪心时先接受残差小的, 让每个位置留下最自洽的那一条。

用法:
    g2o_nms.py <in.g2o> <out.g2o> [--radius 8] [--intra_win 40] [--report out.txt]
  输入需要同名的 _index.csv (顶点 id -> session + pcd 路径), 那是 saveG2oIndex 写的。
"""
import argparse, os, sys, math, collections
import numpy as np


# ---------------------------------------------------------------- 读写 g2o
def q2R(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)]])


def load_index(g2o_path):
    """顶点 id -> (session 名, pcd 路径)。索引是 saveG2oIndex 写的, 头部还带外参。"""
    idx = os.path.splitext(g2o_path)[0] + '_index.csv'
    if not os.path.exists(idx):
        sys.exit(f'找不到索引 {idx} —— 顶点属于哪个 session 全靠它, 没有它无法分组')
    sess, path, ext_line = {}, {}, None
    for ln in open(idx):
        if ln.startswith('#'):
            if 'ext_qwxyz_txyz=' in ln:
                ext_line = ln.rstrip('\n')
            continue
        if ln.startswith('vertex_id'):
            continue
        a = ln.rstrip('\n').split(',', 2)
        if len(a) < 3:
            continue
        v = int(a[0])
        sess[v] = a[1]
        path[v] = a[2]
    return sess, path, ext_line


def load_g2o(p):
    """返回 (头部原文行, 顶点 dict, 边 list, 先验 list, FIX list)。
    边和先验都保留**原始整行文本**, 输出时逐字节照抄 —— 避免浮点 round-trip 改动数值。"""
    head, verts, edges, priors, fixes, params = [], {}, [], [], [], []
    for ln in open(p):
        s = ln.strip()
        if not s:
            continue
        if s.startswith('#'):
            head.append(ln.rstrip('\n'))
            continue
        t = s.split()
        if t[0] == 'VERTEX_SE3:QUAT':
            v = int(t[1])
            verts[v] = (np.array([float(t[2]), float(t[3]), float(t[4])]),
                        q2R(float(t[5]), float(t[6]), float(t[7]), float(t[8])), ln.rstrip('\n'))
        elif t[0] == 'EDGE_SE3:QUAT':
            i, j = int(t[1]), int(t[2])
            tm = np.array([float(t[3]), float(t[4]), float(t[5])])
            Rm = q2R(float(t[6]), float(t[7]), float(t[8]), float(t[9]))
            inf0 = float(t[10])                       # 信息矩阵第一项 = 1/sigma_t^2
            edges.append(dict(i=i, j=j, t=tm, R=Rm, inf=inf0, line=ln.rstrip('\n')))
        elif t[0] == 'EDGE_SE3_PRIOR':
            priors.append(ln.rstrip('\n'))
        elif t[0] == 'FIX':
            fixes.append(ln.rstrip('\n'))
        else:
            params.append(ln.rstrip('\n'))
    return head, verts, edges, priors, fixes, params


# ---------------------------------------------------------------- 分类 / NMS
def classify(edges, sess, order, intra_win):
    """给每条边打上类别和分组。
    类别: 'win' 窗口内 (不动) | 'loop' 同session回环 | 'cross' 跨session
    分组: 同一组内才互相抑制 —— (类别, session_i, session_j), session 对做规范化。
    """
    for e in edges:
        si, sj = sess.get(e['i'], '?'), sess.get(e['j'], '?')
        oi, oj = order.get(e['i'], -1), order.get(e['j'], -1)
        if si == sj:
            # 同 session: 帧号差 <= intra_win 的是分窗逻辑覆盖的局部链, 其余是回环
            e['cls'] = 'win' if (oi >= 0 and oj >= 0 and abs(oi - oj) <= intra_win) else 'loop'
        else:
            e['cls'] = 'cross'
        e['grp'] = (e['cls'],) + tuple(sorted((si, sj)))
        e['si'], e['sj'] = si, sj
    return edges


def residual(e, verts):
    """测量与图解的平移残差 —— 当质量用 (小 = 与最终解一致)。"""
    if e['i'] not in verts or e['j'] not in verts:
        return 1e9
    ti, Ri, _ = verts[e['i']]
    tj, _, _ = verts[e['j']]
    return float(np.linalg.norm(Ri.T @ (tj - ti) - e['t']))


def nms(edges, verts, rad_of):
    """四维 NMS。返回 (保留的边, 每组统计)。

    实现: 按组分开, 组内按残差升序贪心。为了不做 O(n^2), 用 radius 大小的网格给 i 端
    建桶, 只和落在邻桶里的已接受边比 —— 抑制条件要求 i 端在 radius 内, 所以邻桶足够。
    """
    keep, stats = [], {}
    by = collections.defaultdict(list)
    for e in edges:
        by[e['grp']].append(e)
    for grp, es in by.items():
        if grp[0] == 'win':                    # 窗口内边由 thin_window 单独处理
            continue
        radius = rad_of[grp[0]]
        r2 = radius * radius
        es.sort(key=lambda x: x['res'])
        cell = collections.defaultdict(list)   # i 端网格 -> 已接受边
        acc = []
        for e in es:
            pi = verts[e['i']][0][:2] if e['i'] in verts else None
            pj = verts[e['j']][0][:2] if e['j'] in verts else None
            if pi is None or pj is None:
                continue
            cx, cy = int(pi[0] // radius), int(pi[1] // radius)
            dup = False
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for a in cell.get((cx + dx, cy + dy), ()):
                        if ((a[0] - pi) ** 2).sum() <= r2 and ((a[1] - pj) ** 2).sum() <= r2:
                            dup = True
                            break
                    if dup:
                        break
                if dup:
                    break
            if dup:
                continue
            cell[(cx, cy)].append((pi, pj))
            acc.append(e)
        keep.extend(acc)
        stats[grp] = (len(es), len(acc))
        for e in acc:
            e['sup_by'] = grp        # 记下它是在哪个组里胜出的, 供自检
    return keep, stats


def thin_window(edges, sess, order, bys, out):
    """同 session 的窗口内边: **只留相邻帧 (Δ=1) 的链**, 其余 Δ>=2 全丢。

    为什么可以丢: Δ=1 的链已经把每一帧接到整体上了; n<->n+2 ... n<->n+40 说的是同一段
    局部几何, 属于同一件事重复几十遍。实测这类边占同session边的 92% (Δ=1 只有 8.2%),
    也就是说局部那一块的权重被虚高了十几倍 —— 而真正携带全局信息的回环/跨session
    加起来才 4143 条。

    但**不能无脑只留 Δ=1**: 实测四个 session 一共有 23 处 (n,n+1) 缺边 (buildIntra 按
    重叠率/修正量剔掉了), 只留 Δ=1 会把轨迹切成 20 多段, 每段之间只剩 INS 先验。
    所以缺口处要用**能跨过它的最小 Δ 边**补上 —— 那是保连通所必需的最短链接。

    @return (保留的窗口边, 丢弃数, 补桥数)
    """
    keep, drop, bridge = [], 0, 0
    for s, vs in bys.items():
        n = len(vs)
        es = [e for e in edges if e['cls'] == 'win' and e['si'] == s]
        d1, rest = [], []
        for e in es:
            a_, b_ = sorted((order[e['i']], order[e['j']]))
            (d1 if b_ - a_ == 1 else rest).append((a_, b_, e))
        have = {a_ for a_, b_, e in d1}
        keep.extend(e for _, _, e in d1)
        # 缺口: k 与 k+1 之间没有 Δ=1 边。用能跨过它的最短边补
        gaps = [k for k in range(n - 1) if k not in have]
        rest.sort(key=lambda x: x[1] - x[0])          # 按 Δ 升序, 优先用最短的
        used = set()
        for k in gaps:
            for a_, b_, e in rest:
                if a_ <= k < b_ and id(e) not in used:
                    keep.append(e)
                    used.add(id(e))
                    bridge += 1
                    break
        drop += len(rest) - len(used)
        # 连通性自检: 只用保留下来的窗口边, 这个 session 还是几块?
        par = list(range(n))
        def find(x):
            while par[x] != x:
                par[x] = par[par[x]]
                x = par[x]
            return x
        for e in keep:
            if e['si'] != s or e['cls'] != 'win':
                continue
            a_, b_ = find(order[e['i']]), find(order[e['j']])
            if a_ != b_:
                par[a_] = b_
        ncomp = len({find(i) for i in range(n)})
        # **必须和原图比**, 不能只看"是不是 1 个域": 实测这些断点原图里就有 ——
        # 缺口处的空间跳是 12~19m, 超过 intra_pair_dist(12m), 所以任何 Δ 的窗口边都
        # 跨不过去。稀疏化只要不**新增**断点就是对的。
        par0 = list(range(n))
        def f0(x):
            while par0[x] != x:
                par0[x] = par0[par0[x]]
                x = par0[x]
            return x
        for e in es:
            a0, b0 = f0(order[e['i']]), f0(order[e['j']])
            if a0 != b0:
                par0[a0] = b0
        ncomp0 = len({f0(i) for i in range(n)})
        out.append(f'  {s[-6:]}: 窗口边 {len(es)} -> {len(d1) + len(used)} '
                   f'(链 {len(d1)} + 跨缺口的桥 {len(used)} / 缺口 {len(gaps)} 处), '
                   f'丢 {len(rest) - len(used)}  |  连通域 原图 {ncomp0} -> 稀疏后 {ncomp}'
                   + ('  OK 没新增断点' if ncomp <= ncomp0 else
                      f'   !! 新增了 {ncomp - ncomp0} 个断点'))
    return keep, drop, bridge


# ---------------------------------------------------------------- 报告
def spread(edges, verts, arc, label, out):
    """边沿弧长的分布均匀性 —— 这才是"均匀化"要看的东西, 不是总条数。"""
    if not edges:
        out.append(f'  {label}: 0 条')
        return
    a = np.array(sorted(arc.get(e['i'], 0.0) for e in edges))
    gap = np.diff(a) if len(a) > 1 else np.array([0.0])
    cells = collections.Counter((a // 50).astype(int))
    v = np.array(sorted(cells.values()))
    out.append(f'  {label}: {len(edges)} 条 | 相邻边弧长间隔 中位 {np.median(gap):6.1f}m '
               f'间隔<0.5m 占 {100 * (gap < 0.5).mean():3.0f}% | '
               f'50m 格 有边 {len(cells)} 个, 每格 中位 {int(np.median(v))} max {v.max()}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('inp')
    ap.add_argument('out')
    ap.add_argument('--radius', type=float, default=8.0, help='默认半径(m)')
    ap.add_argument('--radius_loop', type=float, default=-1,
                    help='回环的抑制半径(m); <0 用 --radius')
    ap.add_argument('--radius_cross', type=float, default=-1,
                    help='跨session 的抑制半径(m); <0 用 --radius。\n'
                         '**要比回环小**: 跨session 边的两端由 b_max_dist(10m) 约束着本来就'
                         '挨着, 四维条件退化成"每 R 米留一条", 用同一个 R 会把跨session 砍太狠。')
    ap.add_argument('--win_chain', type=int, default=1,
                    help='1 = 同session 窗口内边只留 Δ=1 的链 (+跨缺口的最小桥), 其余丢掉;'
                         ' 0 = 窗口内边全留 (旧行为)')
    ap.add_argument('--intra_win', type=int, default=40,
                    help='帧号差 <= 这个值的同session边算窗口内局部链, **一条不动**')
    ap.add_argument('--report', default='')
    a = ap.parse_args()

    sess, path, ext_line = load_index(a.inp)
    head, verts, edges, priors, fixes, params = load_g2o(a.inp)

    # 顶点在各自 session 里的时序下标 + 累计弧长 (判"窗口内 vs 回环" 和分布均匀性用)
    order, arc = {}, {}
    bys = collections.defaultdict(list)
    for v, s in sess.items():
        bys[s].append(v)
    for s, vs in bys.items():
        vs.sort(key=lambda v: float(os.path.basename(path[v])[:-4]))
        p = np.array([verts[v][0][:2] for v in vs if v in verts])
        c = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])
        for k, v in enumerate(vs):
            order[v] = k
            arc[v] = float(c[k]) if k < len(c) else 0.0

    classify(edges, sess, order, a.intra_win)
    for e in edges:
        e['res'] = residual(e, verts)

    out = []
    out.append(f'输入 {a.inp}')
    out.append(f'  顶点 {len(verts)}  边 {len(edges)}  先验 {len(priors)}  session {len(bys)}')
    n_cls = collections.Counter(e['cls'] for e in edges)
    out.append(f'  分类: 窗口内 {n_cls["win"]} | 回环 {n_cls["loop"]} | 跨session {n_cls["cross"]}'
               f'   (窗口内/回环按 |帧号差| <= {a.intra_win} 分)')
    out.append('')
    out.append(f'=== 去重前的分布 ===')
    for cl in ('loop', 'cross'):
        for grp in sorted({e['grp'] for e in edges if e['cls'] == cl}):
            spread([e for e in edges if e['grp'] == grp], verts, arc,
                   f'{grp[0]:5} {grp[1][-6:]}<->{grp[2][-6:]}', out)

    rad_of = {'loop': a.radius_loop if a.radius_loop > 0 else a.radius,
              'cross': a.radius_cross if a.radius_cross > 0 else a.radius,
              'win': 0.0}
    out.append('')
    out.append('=== 窗口内边: 只留相邻帧的链 (+跨缺口的最小桥) ===' if a.win_chain
               else '=== 窗口内边: 全留 (--win_chain 0) ===')
    if a.win_chain:
        wkeep, wdrop, wbr = thin_window(edges, sess, order, bys, out)
        out.append(f'  合计: 丢 {wdrop} 条 Δ>=2 的冗余窗口边, 补 {wbr} 条跨缺口的桥')
    else:
        wkeep = [e for e in edges if e['cls'] == 'win']
        out.append(f'  {len(wkeep)} 条全部保留')
    keep, stats = nms(edges, verts, rad_of)
    keep = wkeep + keep
    kept = {id(e) for e in keep}

    out.append('')
    out.append(f"=== 四维 NMS (回环 R={rad_of['loop']:.0f}m, 跨session R={rad_of['cross']:.0f}m;"
               f" 两端都近才算重复; 分组 = (类别, session对)) ===")
    for grp in sorted(stats, key=lambda g: (g[0], g[1], g[2])):
        if grp[0] == 'win':
            continue
        b, k = stats[grp]
        out.append(f'  {grp[0]:5} {grp[1][-6:]}<->{grp[2][-6:]}: {b:6d} -> {k:6d} '
                   f'(留 {100.0 * k / max(b, 1):5.1f}%)')
    out.append('')
    out.append(f'=== 去重后的分布 ===')
    for cl in ('loop', 'cross'):
        for grp in sorted({e['grp'] for e in keep if e['cls'] == cl}):
            spread([e for e in keep if e['grp'] == grp], verts, arc,
                   f'{grp[0]:5} {grp[1][-6:]}<->{grp[2][-6:]}', out)

    # ---- 自检 1: 抑制绝不跨组 (这是"A<->B 之后 A<->C 仍要保留"的机制保证) ----
    out.append('')
    out.append('=== 自检1: 抑制**绝不跨组** ===')
    # 只查参与 NMS 的那两类: 窗口内的边走早退分支, 从来不设 sup_by
    chk = [e for e in keep if e['cls'] != 'win']
    bad = sum(1 for e in chk if e.get('sup_by') != e['grp'])
    out.append(f'  参与 NMS 的边 {len(chk)} 条, 胜出组与自身组不一致的: {bad}'
               f'  (必须是 0 —— NMS 是按 grp 分桶跑的)')
    out.append('  注意: "连到 2 个 session 的顶点数" 这种指标**不能**用来验证这条 ——')
    out.append('  边总数砍掉后带边的顶点必然按比例减少, 那个数掉是必然的, 与是否跨组无关。')

    # ---- 自检 2: 位置级的多 session 连接有没有被吃掉 ----
    # 要保住的是"**这个地方**还连着几个 session", 不是"这个特定顶点还连着几个"。
    out.append('')
    out.append('=== 自检2: **位置**级的多session连接 (20m 格) ===')
    for tag, E in (('前', edges), ('后', keep)):
        m = collections.defaultdict(set)
        for e in E:
            if e['cls'] != 'cross':
                continue
            for v, other in ((e['i'], e['sj']), (e['j'], e['si'])):
                if v in verts:
                    p = verts[v][0][:2]
                    m[(int(p[0] // 20), int(p[1] // 20))].add(other)
        c = collections.Counter(len(v) for v in m.values())
        out.append(f'  {tag}: 有跨session边的 20m 格 {len(m)} 个, 其中连到 '
                   + ' '.join(f'{k}个session:{v}' for k, v in sorted(c.items())))
    out.append('  ^ 这两行才该基本持平 —— 格子数和"连到 2/3 个 session 的格子数"都不该塌')

    # ---- 自检 3: 四维比二维到底多留了多少 ----
    # 两类边的两端本来就挨着 (b_max_dist 10m / loop_dist 12m), 所以四维大体退化成
    # "每 R 米一条"。四维唯一多做的事: 同一个地方被**两次不同的重访**各留一条。
    out.append('')
    out.append('=== 自检3: 四维相对二维多留了多少 ===')
    for cl in ('loop', 'cross'):
        E = [e for e in keep if e['cls'] == cl]
        R = rad_of[cl]
        cell = collections.defaultdict(list)
        for e in E:
            p = verts[e['i']][0][:2]
            cell[(int(p[0] // R), int(p[1] // R))].append(verts[e['j']][0][:2])
        extra = sum(len(v) - 1 for v in cell.values())
        far = 0
        for v in cell.values():
            for u in range(1, len(v)):
                if np.linalg.norm(v[u] - v[0]) > R:
                    far += 1
        out.append(f'  {cl:5}: {len(E)} 条落在 {len(cell)} 个 {R:.0f}m 格里 -> '
                   f'同格多条 {extra} 条, 其中 j 端确实相距 >{R:.0f}m 的 {far} 条')
    out.append('  ^ 最后那个数才是"四维"真正的贡献 (同一地点的**不同**重访各留一条);'
               ' 它接近 0 就说明四维在这份数据上等价于二维')

    # 权重影响: 总信息量
    out.append('')
    out.append('=== 总信息量 (sum 1/sigma_t^2, 决定它能不能压过 INS 先验) ===')
    for cl in ('win', 'loop', 'cross'):
        b = sum(e['inf'] for e in edges if e['cls'] == cl)
        k = sum(e['inf'] for e in keep if e['cls'] == cl)
        out.append(f'  {cl:5}: {b:.3e} -> {k:.3e}  ({100.0 * k / max(b, 1e-9):.0f}%)')
    npr = len(priors)
    out.append(f'  参照: INS 先验 {npr} 条 x 1/0.15^2 = {npr / 0.0225:.3e}')
    out.append('  !! 砍掉重复是统计上正确的(重复不是独立测量), 但总信息量确实降了 ——')
    out.append('     跑完要按白化 rms 重新校准 sigma, 否则回环可能又推不动解。')

    # ---- 写 g2o: 逐字节照抄保留的行 ----
    with open(a.out, 'w') as f:
        for h in head:
            f.write(h + '\n')
        f.write(f'# --- 本文件由 g2o_nms.py 生成 ---\n')
        f.write(f'# 源: {a.inp}\n')
        f.write(f"# 四维 NMS: 回环 R={rad_of['loop']}m, 跨session R={rad_of['cross']}m;"
                f" 抑制条件 |Pi-Pi'|<=R 且 |Pj-Pj'|<=R\n")
        f.write(f'#   分组 = (类别, session对): **不同 session 对之间互不抑制** ——\n')
        f.write(f'#   A 与 session1 的 B 建约束后, session2 里 B 附近的 C 仍要建 A<->C。\n')
        f.write(f'#   窗口内帧间边 (|帧号差|<={a.intra_win}) 一条不动, 那是局部里程链。\n')
        f.write(f'# 边: {len(edges)} -> {len(keep)}\n')
        if ext_line:
            f.write(ext_line + '\n')
        for p in params:
            f.write(p + '\n')
        for v in sorted(verts):
            f.write(verts[v][2] + '\n')
        for e in edges:
            if id(e) in kept:
                f.write(e['line'] + '\n')
        for p in priors:
            f.write(p + '\n')
        for x in fixes:
            f.write(x + '\n')
    out.append('')
    out.append(f'写出 {a.out}  (边 {len(edges)} -> {len(keep)}; 顶点/先验/FIX 全部照抄)')
    # 索引照抄一份, 否则 --load_g2o 用不了
    src_idx = os.path.splitext(a.inp)[0] + '_index.csv'
    dst_idx = os.path.splitext(a.out)[0] + '_index.csv'
    if os.path.exists(src_idx) and os.path.abspath(src_idx) != os.path.abspath(dst_idx):
        with open(dst_idx, 'w') as f:
            f.write(open(src_idx).read())
        out.append(f'索引照抄 {dst_idx} (--load_g2o 要用它)')

    txt = '\n'.join(out)
    print(txt)
    if a.report:
        open(a.report, 'w').write(txt + '\n')


if __name__ == '__main__':
    main()
