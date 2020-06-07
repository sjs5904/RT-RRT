# RT-RRT
실시간으로 예측불가능한 움직임을 보이는 목표물을 따라가는 로봇 알고리즘입니다.  
RRT기반 길찾기 알고리즘은 빠르고 점차적으로 최선의 경로를 확보하며 어떤 공간에서든지 사용할 수 있다는 장점이 있습니다.
본 프로젝트는 첨부한 "MIG_2015_paper_6.pdf"의 알고리즘을 기반으로 작성하였습니다.
## RRT star
![3-Figure2-1 - new](https://user-images.githubusercontent.com/49792776/83969472-2ba21b80-a90b-11ea-937d-8f5dfb7c3362.png)  
RRT star는 트리구조에서 더 짧은 경로의 parent를 선택하여 경로를 개선할 수 있습니다.  
## Informed RRT star
