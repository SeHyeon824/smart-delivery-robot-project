# xycar_speaker_server.py - 소켓으로 음성 재생 명령 수신

import socket
import os

# --- 설정값 ---
HOST = ''  # 모든 인터페이스에서 수신
PORT = 9000  # 라즈베리파이에서 이 포트로 명령 전송

# --- 음성 명령 매핑 ---
SOUND_MAP = {
    "START": "01_dochak.mp3",
    "FAIL": "02_fail.mp3",
    "RECOGNIZED": "03_success.mp3",
    "APPROVED": "03_success.mp3",
    "REJECTED": "05_no.mp3"
}

# --- 메인 서버 실행 ---
def main():
    print("🔊 스피커 서버 실행 중... (Ctrl+C로 종료)")

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)

    try:
        while True:
            client_socket, addr = server_socket.accept()
            print(f"📥 연결됨: {addr}")

            data = client_socket.recv(1024).decode().strip()
            print(f"📩 수신된 메시지: {data}")

            if data in SOUND_MAP:
                mp3_path = f"/home/xytron/Downloads/{SOUND_MAP[data]}"
                if os.path.exists(mp3_path):
                    print(f"🔊 {data} → 음성 재생: {mp3_path}")
                    os.system(f"mpg123 \"{mp3_path}\"")
                else:
                    print(f"⚠️ 파일 없음: {mp3_path}")
            else:
                print("⚠️ 알 수 없는 명령")

            client_socket.close()

    except KeyboardInterrupt:
        print("\n서버 종료 요청 감지됨.")
    finally:
        server_socket.close()
        print("소켓 종료 완료.")

if __name__ == '__main__':
    main()
