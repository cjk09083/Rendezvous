# Rendezvous

## 멀티 로봇 랑데부 (Arduino + Zigbee)
<div>
<img src="https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=Arduino&logoColor=white"/>
<img src="https://img.shields.io/badge/Zigbee-EB0443?style=for-the-badge&logo=Zigbee&logoColor=white"/>
</div>

## 목적
- Zigbee를 활용해 군집 네트워크 구축
- 다수의 자식 로봇이 하나의 부모 로봇에게 랑데부
- 이동중 장애물 회피
- 로봇 고장 감지 및 대응

## 담당
- Zigbee 트리 네트워크 구축 
- RSSI 측정을 통한 부모 로봇 탐지
- 부모 로봇 랑데부 및 트리 구조 재구축
- 초음파 센서를 이용한 장애물 회피

## 기능

### Zigbee 트리구조 구축
- 아두이노와 Xbee 모듈을 이용하여 트리 네트워크 구축
<div align="center">
<img src="https://github.com/cjk09083/Rendezvous/blob/main/사진%26영상/Xbee.jpg" width="30%"/> &nbsp;
<img src="https://github.com/cjk09083/Rendezvous/blob/main/사진%26영상/XCTU.jpg" width="30%"/> &nbsp;
<img src="https://github.com/cjk09083/Rendezvous/blob/main/사진%26영상/Zigbee%20네트워크.jpg" width="30%"/> &nbsp;
</div>


### 부모 로봇 탐지
- 로봇 좌우에 Xbee 모듈을 장착해 네트워크로 연결된 부모로봇과의 거리를 RSSI로 측정
- 두 RSSI값이 유사해지도록 차체를 회전해 부모로봇을 바라보게 만듬
<div align="center">
<img src="https://github.com/cjk09083/Rendezvous/blob/main/사진%26영상/로봇%20조감도.png" width="60%"/> &nbsp;
</div>



