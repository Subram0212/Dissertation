if 7000//120 <= i <= 37200//120:
    # Your code for setting dynamic_locs...
    if dynamic_node_flag == 1:
        scn_map2, = ax.plot(7.2*0.22526, 6*0.22526, 'ro', markeredgewidth=2, markersize=5)
        if scn_map5 is None:
            scn_map5, = ax.plot(6.5*0.22526, 1.5*0.22526, 'ko', markeredgewidth=2, markersize=5)
    elif dynamic_node_flag == 2:
        scn_map2, = ax.plot(6.5*0.22526, 1.5*0.22526, 'ro', markeredgewidth=2, markersize=5)
        if scn_map6 is None:
            scn_map6, = ax.plot(7.2*0.22526, 6*0.22526, 'ro', markeredgewidth=2, markersize=5)
        # Plot with 'ro' if necessary, but do not reassign scn_map6
    else:
        scn_map2, = ax.plot(dynamic_locs_x, dynamic_locs_y, 'ko', markeredgewidth=2, markersize=5)

elif i >= 37500//120:
    # Remove scn_map5 and scn_map6 if they were created
    if scn_map5:
        scn_map5.remove()
        scn_map5 = None
    if scn_map6:
        scn_map6.remove()
        scn_map6 = None