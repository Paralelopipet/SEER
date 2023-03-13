def with_slope(*, env_params, **_):
    env_params.update(
        gravity_angle = 0.3,
        tip_penalty = -30.0,  # -20.0,
        tipping_threshold=0.5
    )
