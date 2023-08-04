# Micromouse
This project was done for participating in International Micromouse Competition organized by
[TechFest2019-20 at IIT Bombay](https://techfest.org/). It is
the furtherance of the Tremaux algorithm by employing a potential value algorithm in conjunction, with improving search in a micromouse.  The fused algorithm running on the STM32 Bluepill microcontroller explores and finds the shortest path in a 16x16 maze. Proximity sensors, gyroscope, and magnetometer together with encoder motors aid the micromouse to understand its surrounding hindrances and make precise movements while traversing through a maze. The optimized algorithm eliminates any paths that may lead the micromouse further away from the center of the maze during the initial run itself and saves a significant amount of time while solving a maze.

## Flow Chart of the Optimized Algorithm

![flowchart](/Figures/flow_chart.png)

Here are the three different versions of micromouse designed and fabricated. The bot at the center was used during the IMC-2020.
![bots](/Figures/bots.png)

## Citation:
You can find our publication [here](https://ictaes.org/wp-content/uploads/2020/09/IJAE-2020-Vol.03-No.02/7_Sanjaya_Vol3_No2.pdf?ckattempt=1).
```
@article{rijalinternational,
  title={Optimizing Tremaux Algorithm in Micromouse Using Potential Values},
  journal={International Journal of Advanced Engineering},
  author={Rijal, Sanjay and Nepal, Rabin and Lwagun, Rhimesh and Pati, Rohit and Bhatta, Janardan},
  volume={3},
  issue={2},
  year={2020}
}

```
