# Trajectory planning problem for detecting radioactive sources using underwater gliders

This work is the master's thesis of Mr. Ibrahim, Ahmed Yehia Zakaria @ Instituto Superior Tecnico. This work has been upgraded and published in the  [Journal of Marine Science and Engineering](https://www.mdpi.com/2077-1312/13/7/1369) in July 2025.
![results](https://github.com/user-attachments/assets/9135329c-b352-4c2e-a81e-fac08f94de52)
        The coverage trajectory in different map cities is shown, ensuring all areas within the boundary are visited.


The abstract is as follows:

In many applications, including underwater robotics, the coverage problem requires an autonomous vehicle to systematically explore a defined area while minimizing redundancy and avoiding obstacles. This paper investigates coverage path-planning strategies to enhance the efficiency of underwater gliders, particularly in maximizing the probability of detecting a radioactive source while ensuring safe navigation. We evaluate three path-planning approaches: the Traveling Salesman Problem (TSP), Minimum Spanning Tree (MST), and the Optimal Control Problem (OCP). Simulations were conducted in MATLAB R2020a, comparing processing time, uncovered areas, path length, and traversal time. Results indicate that the OCP is preferable when traversal time is constrained, although it incurs significantly higher computational costs. Conversely, MST-based approaches provide faster but fewer optimal solutions. These findings offer insights into selecting appropriate algorithms based on mission priorities, balancing efficiency and computational feasibility.

Keywords: optimal control; covering problem; path planning; search optimization

## **Funding**
This work was supported by European Commission via Erasmus Mundus Joint Masters Degrees (EMJMD) “_Marine and Maritime intelligent robotics_” and Fundação para a Ciência e Tecnologia under the projects UIDB/04111/2020 and CTS/00066.

## **Acknowledgments**

This work was inspired by the pioneering research of Isaac Kaminer and António Pascoal, which guided the development of the optimal-control approach adopted herein.

## **Citation**

If you use this work for your research, please cite:
```bibtex
@article{ibrahim2025comparison,
  title={Comparison of innovative strategies for the coverage problem: Path planning, search optimization, and applications in underwater robotics},
  author={Ibrahim, Ahmed and Rego, Francisco FC and Busvelle, {\'E}ric},
  journal={Journal of Marine Science and Engineering},
  volume={13},
  number={7},
  pages={1369},
  year={2025},
  publisher={MDPI}
}
