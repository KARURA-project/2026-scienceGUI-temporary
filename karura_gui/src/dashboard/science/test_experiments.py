"""
Test Experiment Dashboard UI (Hybrid Mode)

このスクリプトは以下のハイブリッド動作を行います:
- ビデオソースとその他テレメトリ (バッテリー, GPS, IMU, Stepper): 実際のROS 2トピックを受信
- センサーデータ (Temp, Hum, Press, UV, CO2, TVOC, HCHO, NH3): モックデータ(ダミー)を生成
"""

import sys
import os
import random
from pathlib import Path
from PySide6.QtWidgets import QApplication, QMessageBox
from PySide6.QtCore import QTimer

# プロジェクトのルートディレクトリをパスに追加
project_root = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path.insert(0, project_root)

# ROS 2のモジュールをインポート (本物のカメラと通信するため必須)
try:
    from std_msgs.msg import Float64
except ImportError:
    print("[ERROR] ROS 2モジュールが見つかりません。 'source /opt/ros/jazzy/setup.bash' を実行してください。")
    sys.exit(1)

from dashboard.science.window import ScienceMainWindow
from dashboard.science.bridge import ScienceBridge


class SmoothSensorData:
    """滑らかなダミーセンサーデータを生成するクラス"""
    def __init__(self):
        self.values = {
            "Temperature": 25.0, "Humidity": 50.0, "Pressure": 1010.0,
            "UV": 3.00, "CO2": 600.0, "TVOC": 300.0, "HCHO": 1.0, "NH3": 0.10, "HCN": 0.005
        }
        self.deltas = {
            "Temperature": 0.2, "Humidity": 0.5, "Pressure": 0.5,
            "UV": 0.05, "CO2": 2.0, "TVOC": 3.0, "HCHO": 0.05, "NH3": 0.01, "HCN": 0.001
        }
        self.limits = {
            "Temperature": (15.0, 35.0), "Humidity": (30.0, 70.0), "Pressure": (990.0, 1030.0),
            "UV": (1.50, 5.00), "CO2": (500.0, 700.0), "TVOC": (200.0, 450.0),
            "HCHO": (0.0, 2.0), "NH3": (0.00, 0.20), "HCN": (0.00, 0.01)
        }

    def get_next(self):
        for k in self.values:
            change = random.uniform(-self.deltas[k], self.deltas[k])
            self.values[k] = max(self.limits[k][0], min(self.limits[k][1], self.values[k] + change))
        return self.values


def simulate_sensors(window, sensor_sim):
    """センサーデータのみをダミーで更新する関数 (他のUIは触らない)"""
    data = sensor_sim.get_next()

    def make_f64(val):
        msg = Float64()
        msg.data = float(val)
        return msg

    window.on_temperature_update(make_f64(data["Temperature"]))
    window.on_humidity_update(make_f64(data["Humidity"]))
    window.on_pressure_update(make_f64(data["Pressure"]))
    window.on_uv_update(make_f64(data["UV"]))
    window.on_co2_update(make_f64(data["CO2"]))
    window.on_voc_update(make_f64(data["TVOC"]))
    window.on_hcho_update(make_f64(data["HCHO"]))
    window.on_nh3_update(make_f64(data["NH3"]))
    window.on_hcn_update(make_f64(data["HCN"]))




def main():
    print("Starting Science Dashboard (Hybrid Test Mode)...")
    app = QApplication(sys.argv)

    # スタイルシートの読み込み
    qss_path = Path(__file__).resolve().parent.parent / "core" / "karura_dark.qss"
    if qss_path.exists():
        app.setStyleSheet(qss_path.read_text(encoding="utf-8"))
        print(f"[STYLE] Loaded QSS: {qss_path}")

    try:
        # 本物のROS 2ブリッジを起動 (カメラ・GPS・IMU・バッテリー等を受信)
        bridge = ScienceBridge()
        bridge.start()

        # ウィンドウの作成と全信号の接続
        window = ScienceMainWindow()
        window.connect_signals(bridge)

        # 【重要】本物のセンサーデータが混ざらないように、センサーの信号だけ切断する
        try:
            bridge.temperature_signal.disconnect(window.on_temperature_update)
            bridge.humidity_signal.disconnect(window.on_humidity_update)
            bridge.pressure_signal.disconnect(window.on_pressure_update)
            bridge.uv_signal.disconnect(window.on_uv_update)
            bridge.co2_signal.disconnect(window.on_co2_update)
            bridge.voc_signal.disconnect(window.on_voc_update)
            bridge.hcho_signal.disconnect(window.on_hcho_update)
            bridge.nh3_signal.disconnect(window.on_nh3_update)
            bridge.hcn_signal.disconnect(window.on_hcn_update)

        except Exception as e:
            print(f"[WARN] センサー信号の切断に失敗しました: {e}")

        # ダミーセンサーデータの準備
        sensor_sim = SmoothSensorData()

        # 1000ミリ秒ごとにダミーセンサーデータだけをUIに流し込むタイマー
        timer = QTimer()
        timer.timeout.connect(lambda: simulate_sensors(window, sensor_sim))
        timer.start(1000)

        window.show()
        window.move(100, 100)
        window.raise_()
        window.activateWindow()

        print("======================================================")
        print("  Hybrid Dashboard Started Successfully!              ")
        print("  - Video/IMU/GPS/Battery : REAL (via ROS 2)          ")
        print("  - Science Sensors       : MOCK (Smooth Dummy Data)  ")
        print("======================================================")

        # 終了時にROS 2ノードを安全にシャットダウン
        app.aboutToQuit.connect(bridge.shutdown)

        sys.exit(app.exec())

    except Exception as e:
        print(f"Error starting hybrid dashboard: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()