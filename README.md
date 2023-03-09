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

`python pybullet_multigoal_gym/pybullet_multigoal_gym/examples/kuka_tip_over.py`

## Debug Simulation in VSCode

Press `F5` (launch configuration `Tip Over Simulation`)

### Codestyle

- Please use 4 space indentation.

## Run with Docker

1. Build: `docker build -f evaluate.Dockerfile -t seer-evaluate .`
2. Run: `wandb docker-run --memory=4g --cpus=3  --mount type=bind,source=<path to COPIED examples folder>,target=/seer/pybullet_multigoal_implementation/drl_implementation/examples --mount type=bind,source=<path to COPIED evaluation_tools folder>,target=/seer/seer/evaluation_tools seer-evaluate` 
  - note that you need to replace `<path to COPIED examples folder>` by the absolute path to the examples folder you want to use (including the saved model checkpoints)
  - note that you need to replace `<path to COPIED evaluation_tools folder>` by the absolute path to the evaluation_tools folder you want to use (including the configuration for if should use training / evaluation etc.)
  - example: `wandb docker-run --memory=4g --cpus=3  --mount type=bind,source=C:\Users\mgraf\programming\cambridge\l32\SEER\pybullet_multigoal_implementation\drl_implementation\examples2,target=/seer/pybullet_multigoal_implementation/drl_implementation/examples --mount type=bind,source=C:\Users\mgraf\programming\cambridge\l32\SEER\seer\evaluation_tools2,target=/seer/seer/evaluation_tools seer-evaluate`
