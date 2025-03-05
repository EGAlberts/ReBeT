# ReBeT
Reconfiguration with Behavior Trees.

This is a library for ROS2 Humble which adds two new types of BT nodes to BT.CPP/BT.ROS2: QRDecorators and AdaptDecorators.
The former makes it possible to explicitly represent quality requirements, and reason against these with BT logic, or logic specific in AdaptDecorators.
The latter provides a hook for adapting ROS2 nodes, changing their parameters, lifecycle states, or topics. 

We have written a paper about ReBeT published at [ACSOS 2024](https://2024.acsos.org/) which you can read [here](https://egalberts.github.io/publication/alberts-rebet-2024/alberts-rebet-2024.pdf).

You can cite it as follows:

```
@inproceedings{alberts2024rebet,
  title={ReBeT: Architecture-based Self-adaptation of Robotic Systems through Behavior Trees},
  author={Alberts, Elvin and Gerostathopoulos, Ilias and Stoico, Vincenzo and Lago, Patricia},
  booktitle={2024 IEEE International Conference on Autonomic Computing and Self-Organizing Systems (ACSOS)},
  pages={1--10},
  year={2024},
  organization={IEEE}
}
```

ReBeT is only a library, which means it cannot do much on its own. You can find an example of how ReBeT can be used in the rebet_frog respository [here](https://github.com/EGAlberts/rebet_frog).