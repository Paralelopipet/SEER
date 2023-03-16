# SEER

SEER - Combining Analytic Control with Learning to Create a Stabilizing Controller that Works in Reality

Built by Damjan Denic, Martin Graf, Nav Leelarathna, and Sepand Dyanatkar

## Setup

1. Clone the repository inclusive submodules `git clone --recurse-submodules git@github.com:Paralelopipet/SEER.git`
2. Pretrained weights are available [here](https://drive.google.com/drive/folders/1jb2BDK5zjfsgHRM_pPp3TxM13kClO0f4?usp=sharing).
  - If you want to use them, place them in the folder `pybullet_multigoal_implementation/drl_implementation/examples`
3. To setup natively:
  1. Create conda environment `conda env create -f environment.yml`
    - note: this will take a while (3 to 4 coffees)
  2. Activate environment `conda activate l32_seer`
  3. Install our gym package `pip install --editable pybullet_multigoal_gym`
  4. Install our RL package `pip install --editable pybullet_multigoal_implementation`
  5. Install seer package `pip install --editable .`
4. To setup with Docker:
  1. Run `docker build -f evaluate.Dockerfile -t seer-evaluate .`
    - this needs to be repeated whenever any files were changed

## Test (requires native install)

Run `pytest`

## Train

- to train natively, run
`python seer/train_and_eval_configs/config_runner.py --config seer.train_and_eval_configs.rl_training.<scenario to train>`
- To train using Docker, run `wandb docker-run --memory=6g --cpus=4 --mount "type=bind,source=$PWD/pybullet_multigoal_implementation/drl_implementation/examples,target=/root/pybullet_multigoal_implementation/drl_implementation/examples" -it seer-evaluate --config seer.train_and_eval_configs.rl_training.<scenario to train>`
  - in CMD, replace `$PWD` by the absolute path of this directory
- **In both cases**, replace `<scenario to train>` with the name of the Python file containing the scenario you want to train (i.e., `rl_config_train_basic`)
- weights are saved to the `pybullet_multigoal_implementation/drl_implementation/examples` folder


## Evaluate
- to evaluate natively, run
`python seer/train_and_eval_configs/config_runner.py --config seer.train_and_eval_configs.<scenario to evaluate>`
- To train using Docker, run `wandb docker-run --memory=4g --cpus=2 --mount "type=bind,source=$PWD/pybullet_multigoal_implementation/drl_implementation/examples,target=/root/pybullet_multigoal_implementation/drl_implementation/examples" -it seer-evaluate --config seer.train_and_eval_configs.<scenario to evaluate>`
  - in CMD, replace `$PWD` by the absolute path of this directory
- **In both cases**, replace `<scenario to evaluate>` with the partial package name of the Python file containing the scenario you want to evaluate (i.e., `rl_eval.basic.rl_config_eval_basic` or `baseline.baseline_config_noisy_slope`)
- if evaluating the reinforcement learning solution, weights need to be present in the `pybullet_multigoal_implementation/drl_implementation/examples` folder

## Debug in VSCode

- press `F5` (launch configuration `RL Trainer`)
- or try out the other launch configurations in `.vscode/launch.json`
