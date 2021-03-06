# RT-RRT
실시간으로 예측불가능한 움직임을 보이는 목표물을 따라가는 로봇 알고리즘입니다.  
RRT기반 길찾기 알고리즘은 빠르고 점차적으로 최선의 경로를 확보하며 어떤 공간에서든지 사용할 수 있다는 장점이 있습니다.
본 프로젝트는 첨부한 "MIG_2015_paper_6.pdf"의 알고리즘을 기반으로 작성하였습니다.
## RRT star
![3-Figure2-1 - new](https://user-images.githubusercontent.com/49792776/83969472-2ba21b80-a90b-11ea-937d-8f5dfb7c3362.png)  
RRT star는 트리구조에서 더 짧은 경로의 parent를 선택하여 경로를 개선할 수 있습니다.  
### Rewiring
![4](https://user-images.githubusercontent.com/49792776/83969987-fc40de00-a90d-11ea-984a-9b60e85ee100.PNG)  
![5](https://user-images.githubusercontent.com/49792776/83969995-0662dc80-a90e-11ea-84dd-1042f64ba847.PNG)  
재배열은 정해진 시간이 지날때까지 옆 노드로 퍼져갑니다.  
root에서부터 재배열 할 경우 root에서 점점 멀어지게 퍼집니다.  
## Informed RRT star
![informedrrt](https://user-images.githubusercontent.com/49792776/83969712-707a8200-a90c-11ea-91a1-756242717796.PNG)  
Informed RRT star는 현재위치와 목표위치를 초점으로 둔 타원의 안에 집중적으로 샘플을 생성합니다.  
코드의 실행이 지나치게 느린 관계로 이 방법은 제 프로젝트에서 제외되었습니다.
## 경로 지정하기
![6](https://user-images.githubusercontent.com/49792776/83970270-cb61a880-a90f-11ea-88a2-b1d52bec57d0.PNG)  
벽과 마주치거나 가능성이 낮은 경로는 비용을 최대로 설정해 배제합니다.  
목표 위치와 연결될 샘플 노드가 존제하지 않는다면  가장 가까운 샘플 노드를 향해 경로를 설정합니다.  
## 결과
![14241242142142142](https://user-images.githubusercontent.com/49792776/83970376-665a8280-a910-11ea-95c3-d4b8ff5df18f.PNG)  
목표물의 움직임을 어떻게 설정하던지간에 로봇은 공간 전체에 깔린 샘플 노드들을 기반으로 계속적으로 목표물을 추적합니다.  
