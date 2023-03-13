def with_noise(*, env_params, algo_params, **_):
    env_params.update(
        noise_stds = {
            'pos' : 100.0,
            'vel' : 100.0,
            'tor' : 100.0,
            'com' : 100.0
        }
    )
    algo_params.update(
        action_noise_std = 1.0,
        observation_noise_std = env_params['noise_stds']
    )
