

# for all v in V:
    # get new position based on forward simulation
    # pos_new = forward(v, pos_now, t)

    # check collision with static features in environment
    # collision = check_collision(pos_new, ws_model)

    # if collision:
        # suitable_v.append(v)

#### for forward function

    # p_new = p_old + v * t // would t be fixed and what would it's value?

#### for check_collision function

    # if we're using a wall (i.e. a line in the 2D space), we can simply check this
    # get an equation for the line
    # 1. check status of pose_now:
    #   if a(x_now) + b(y_now) > c:
    #       status = smaller
    #   else:
    #       status = greater

    # 2. check status of pose_new:
    #   if a(x_new) + b(y_new) > c:
    #       new_status = smaller
    #   else:
    #       new_status = greater

    # 3. compare both statuses:
    #   if new_status == status:
    #       collide = false
    #   else:
    #       collide = true