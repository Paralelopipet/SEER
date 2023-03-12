# SEER

SEER - Stabilizing Energy Efficient Controller Manipulators using Reinforcement Learning

Built by Damjan Denic, Martin Graf, Nav Leelarathna, and Sepand Dyanatkar

## Setup

1. Clone the repository inclusive submodules `git clone --recurse-submodules git@github.com:Paralelopipet/SEER.git`
2. Create conda environment `conda env create -f environment.yml`
    - note: this will take a while (3 to 4 coffees)
3. Activate environment `conda activate l32_seer`
4. Install our gym package `pip install --editable pybullet_multigoal_gym`
5. Install our RL package `pip install --editable pybullet_multigoal_implementation`
6. Install seer package `pip install --editable .`

TODO Sometimes you need to run `git submodule update --recursive --remote` (Why?)

## Test

Run `pytest`

## Run Simulation

`python pybullet_multigoal_gym/pybullet_multigoal_gym/examples/kuka_tip_over.py --use-wandb --config "seer.train_and_eval_configs.rl_eval.rl_config_eval_basic"`

## Debug Simulation in VSCode

Press `F5` (launch configuration `Tip Over Simulation`)
Or try out the other launch configurations in `.vscode/launch.json`

### Codestyle

- Please use 4 space indentation.

## Run with Docker

1. Build: `docker build -f evaluate.Dockerfile -t seer-evaluate .`
2. Run: `wandb docker-run --memory=4g --cpus=3 --mount type=bind,source=<path to your examples folder>,target=/root/pybullet_multigoal_implementation/drl_implementation/examples -it seer-evaluate --use_wandb --config "seer.train_and_eval_configs.rl_eval.rl_config_eval_basic"` 
  - note that you need to replace `<path to your examples folder>` by the absolute path to the examples folder (that include the saved model checkpoints)
  - This means that during build we are copying everything EXCEPT the examples folder. We are essentially symlinking the examples folder from the host to the container.

(If you have a lot of failed builds, be careful that docker doesn't take up too much space. Many useful commands out there, here's an example of some: [link](https://stackoverflow.com/questions/39878939/docker-filling-up-storage-on-macos))