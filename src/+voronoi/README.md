# Voronoi Route Planning

Source: `Malye_Bespilotnye_Letatelnye_Apparaty_Teoria_I_Praktika_2015.pdf`,
section 12.1.1, pp. 217-222, Algorithm 9, Eq. (12.1), Fig. 12.5.

The planner samples rectangular obstacle perimeters with spacing no greater than
`map.resolution`, constructs the point Voronoi graph using the dual Delaunay
triangulation, and removes infinite or collision-producing edges. Finite edges
are tested against closed rectangles inflated by `safetyRadius` using exact
segment intersection, rather than collision samples.

For this bounded workspace, map perimeter samples are also included in the site
set Q. This is a project adaptation: map walls participate in both the diagram
and the clearance cost, make sparse obstacle scenarios usable without adding
random obstacles, and do not alter the scenario geometry. Each wall has at
least three samples, ensuring Algorithm 9's minimum of ten sites even with
coarse map resolution. The representation
approximates extended obstacle boundaries through perimeter samples as in
Fig. 12.5; it is not an analytic segment-site Voronoi solver.

Start and goal are added to the graph and connected to the three nearest
remaining Voronoi vertices. Intersecting connections are removed. No further
vertices or direct start-goal shortcuts are substituted if these connections
fail. Edges receive the book's cost:

`J(a,b) = k1 * norm(b-a) + k2 / minDistance(segment(a,b), Q)`.

Point-to-segment distance uses the projection parameter clamped to `[0,1]`.
`graph` and `shortestpath(...,'Method','positive')` perform Dijkstra search.
Default weights `k1=0.1`, `k2=0.9` match the example on p. 220; change them
through `scenario.voronoi.k1` and `scenario.voronoi.k2`.

The returned `rawPath` follows the graph. The experiment's subsequent shortcut
and minimum-snap stages can leave graph edges; the book's edge cost describes
the raw route only. The graph, its weights, clearances, connections, route node
IDs and route cost are available in `result.tree`. A disconnected graph returns
an empty route with a diagnostic `message`.

Verification: `runtests('tests/testVoronoi.m')` and `run('scripts/run_demo.m')`.
