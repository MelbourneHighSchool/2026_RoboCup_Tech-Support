def bot(
    x_pos,
    y_pos,
    yaw,
    ball_x,
    ball_y,
    ball_captured=False,
    state=None,
    friendly_bot_positions=None,
    enemy_bot_positions=None,
):
    """A simple bot that does nothing. Used for testing the simulation and visualisation."""
    return None, 0, yaw, None, False, 0