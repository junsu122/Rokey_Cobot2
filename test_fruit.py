# import cv2
# import numpy as np
# import pyrealsense2 as rs
# from ultralytics import YOLO

# def main():
#     # 1. YOLO 모델 로드 (가져오신 .pt 파일 경로를 입력하세요)
#     # model = YOLO("/home/junsu/yolov8n.pt") 
#     model = YOLO("/home/junsu/Rokey_Cobot2/rokey_fruit.pt")

#     # 2. RealSense 파이프라인 설정
#     pipeline = rs.pipeline()
#     config = rs.config()

#     # 컬러 스트림 설정 (640x480, 30fps)
#     config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

#     # 스트리밍 시작
#     pipeline.start(config)

#     print("과일 인식 프로그램을 시작합니다. (종료하려면 'q'를 누르세요)")

#     try:
#         while True:
#             # RealSense로부터 프레임 세트 가져오기
#             frames = pipeline.wait_for_frames()
#             color_frame = frames.get_color_frame()
#             if not color_frame:
#                 continue

#             # 프레임을 numpy 배열로 변환
#             img = np.asanyarray(color_frame.get_data())

#             # 3. YOLO 추론 수행
#             # conf=0.5는 신뢰도가 50% 이상인 것만 표시합니다.
#             results = model.predict(source=img, conf=0.5, save=False)

#             # 결과 시각화 (results[0].plot()은 바운딩 박스가 그려진 이미지를 반환합니다)
#             annotated_frame = results[0].plot()

#             # 화면에 출력
#             cv2.imshow("RealSense Fruit Detection", annotated_frame)

#             # 'q' 키를 누르면 종료
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 break

#     finally:
#         # 파이프라인 및 창 닫기
#         pipeline.stop()
#         cv2.destroyAllWindows()

# if __name__ == "__main__":
#     main()


import cv2
from ultralytics import YOLO

def main():
    # 1. 준수님이 학습시킨 YOLO 모델 불러오기
    # 💡 GPU를 사용하려면 뒤에 .to('cuda')를 붙이거나 device=0을 설정합니다.
    model = YOLO("/home/junsu/Rokey_Cobot2/rokey_fruit.pt")

    # 2. 일반 웹캠 연결 (0번은 기본 내장 웹캠, 외장 웹캠은 1 또는 2)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("⚠️ 웹캠을 열 수 없습니다. 연결을 확인해 주세요.")
        return

    # 웹캠 해상도 설정 (원하는 크기로 조절 가능)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("🎥 웹캠 가동 중... 종료하려면 키보드의 'q'를 누르세요.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️ 프레임을 읽어올 수 없습니다.")
            break

        # 3. YOLO 모델로 객체 감지 실행 (device=0으로 GPU 강제 지정)
        results = model.predict(source=frame, device=0, conf=0.5, verbose=False)

        # 4. 감지된 결과를 화면에 그리기
        annotated_frame = results[0].plot()

        # 화면에 결과 영상 출력
        cv2.imshow("YOLOv8 WebCam Test", annotated_frame)

        # 'q' 키를 누르면 루프 탈출 및 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 자원 해제
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()