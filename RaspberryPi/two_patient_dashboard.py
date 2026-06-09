import sys
import ast
import threading
from collections import deque
from datetime import datetime
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import serial
import time

ERROR_INFO = {
    1: ("HIGH HEART RATE", "Heart rate above normal range.", "#e60000"),
    2: ("LOW HEART RATE", "Heart rate below normal range.", "#e60000"),
    3: ("TEMP SPIKE", "Temperature above normal range.", "#e66a00"),
    4: ("TEMP DROP", "Temperature below normal range.", "#e66a00"),
    5: ("LOW SPO2", "SpO2 level below normal range.", "#9b00b5"),
    6: ("FALL DETECTED", "Fall detected from motion sensors.", "#cc0000"),
}


class VitalsDashboard(QtWidgets.QWidget):
    new_packet = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("VitalsPatch Monitor - Phase 2")
        self.resize(1300, 850)

        self.current_alerts = {1: [], 2: []}
        self.patient_data = {}

        self.main_layout = QtWidgets.QVBoxLayout(self)

        self.alert_area = QtWidgets.QVBoxLayout()
        self.main_layout.addLayout(self.alert_area)

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setTabPosition(QtWidgets.QTabWidget.South)
        self.tabs.setStyleSheet("QTabBar::tab { height: 40px; width: 200px; font-size: 18px; }")
        self.main_layout.addWidget(self.tabs)

        for patient_id in [1, 2]:
            self.setup_patient_tab(patient_id)

        self.refresh_global_alert_banners()

        self.new_packet.connect(self.process_packet)

        print("\nVitalsPatch Phase 2 console-input UI is running.")
        print("Paste packets like this:")
        print("Patient:1;HR:[75,76,74];SPO2:[98,99,97];TEMP:[98.4,98.5,98.3];ERR:[0,0,0,0,0,0]\n")

        threading.Thread(target=self.console_reader, daemon=True).start()

    def setup_patient_tab(self, patient_id):
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)

        title = QtWidgets.QLabel(f"Patient {patient_id} Vitals")
        title.setStyleSheet("font-size: 24px; font-weight: bold; padding: 8px;")
        layout.addWidget(title)

        hr_plot = self.make_plot("Heart Rate (bpm)", 40, 160)
        spo2_plot = self.make_plot("SpO2 (%)", 80, 101)
        temp_plot = self.make_plot("Temperature (°F)", 94, 105)

        hr_curve = hr_plot.plot(pen=pg.mkPen("#ff0000", width=2), symbol="o", symbolSize=5)
        spo2_curve = spo2_plot.plot(pen=pg.mkPen("#00d9ff", width=2), symbol="o", symbolSize=5)
        temp_curve = temp_plot.plot(pen=pg.mkPen("#ffff00", width=2), symbol="o", symbolSize=5)

        layout.addWidget(hr_plot)
        layout.addWidget(spo2_plot)
        layout.addWidget(temp_plot)

        log_title = QtWidgets.QLabel(f"PATIENT {patient_id} ALERT LOG")
        log_title.setStyleSheet("font-size: 18px; font-weight: bold; padding: 6px;")
        layout.addWidget(log_title)

        alert_log = QtWidgets.QListWidget()
        alert_log.setStyleSheet("font-size: 15px;")
        layout.addWidget(alert_log)

        self.patient_data[patient_id] = {
            "hr_data": deque(maxlen=120),
            "spo2_data": deque(maxlen=120),
            "temp_data": deque(maxlen=120),
            "hr_curve": hr_curve,
            "spo2_curve": spo2_curve,
            "temp_curve": temp_curve,
            "alert_log": alert_log,
        }

        self.tabs.addTab(tab, f"Patient {patient_id}")

    def make_plot(self, title, ymin, ymax):
        plot = pg.PlotWidget(title=title)
        plot.setBackground("k")
        plot.setYRange(ymin, ymax)
        plot.showGrid(x=True, y=True)
        plot.setLabel("bottom", "Sample")
        return plot

    def make_banner(self, title, subtitle, color):
        label = QtWidgets.QLabel(f"{title}\n{subtitle}")
        label.setStyleSheet(
            f"""
            background-color: {color};
            color: white;
            font-size: 22px;
            font-weight: bold;
            padding: 12px;
            border-radius: 4px;
            """
        )
        label.setMinimumHeight(75)
        return label

    def clear_alert_banners(self):
        while self.alert_area.count():
            item = self.alert_area.takeAt(0)
            widget = item.widget()
            if widget:
                widget.deleteLater()

    def refresh_global_alert_banners(self):
        self.clear_alert_banners()

        any_alerts = False

        for patient_id in [1, 2]:
            for code in self.current_alerts[patient_id]:
                any_alerts = True
                title, subtitle, color = ERROR_INFO.get(
                    code,
                    (f"UNKNOWN ERROR {code}", "Unknown alert code received.", "#555555")
                )

                banner = self.make_banner(
                    f"PATIENT {patient_id}: {title} (Code {code})",
                    subtitle,
                    color
                )
                self.alert_area.addWidget(banner)

        if not any_alerts:
            self.alert_area.addWidget(
                self.make_banner(
                    "SYSTEM STATUS",
                    "Monitoring patients. No current alerts.",
                    "#138a13"
                )
            )

    def console_reader(self):
        #For wireless bluetooth communication:
        # port = "/dev/ttyACM0"
        # baud_rate = 115200

        # while True:
        #     try:
        #         print(f"Connecting to {port}...")

        #         ser = serial.Serial(
        #             port=port,
        #             baudrate=baud_rate,
        #             timeout=1
        #         )

        #         print("Connected to STM32 USB CDC device")

        #         ser.reset_input_buffer()

        #         while True:

        #             if ser.in_waiting > 0:

        #                 packet = ser.readline() \
        #                     .decode("utf-8", errors="ignore") \
        #                     .strip()

        #                 if packet:
        #                     print("Received:", packet)
        #                     self.new_packet.emit(packet)

        #             time.sleep(0.01)

        #     except serial.SerialException as e:
        #         print("Serial Error:", e)
        #         time.sleep(2)

        #for console testing: 
        while True:
           packet = input("Enter packet: ")
           self.new_packet.emit(packet)

    



       

    def parse_packet(self, packet):
        parsed = {}

        for part in packet.strip().split(";"):
            key, value = part.split(":", 1)
            key = key.strip().upper()
            value = value.strip()

            if key == "PATIENT":
                parsed[key] = int(value)
            else:
                parsed[key] = ast.literal_eval(value)

        return (
            parsed["PATIENT"],
            parsed["HR"],
            parsed["SPO2"],
            parsed["TEMP"],
            parsed["ERR"],
        )

    def process_packet(self, packet):
        try:
            patient_id, hr_values, spo2_values, temp_values, err_codes = self.parse_packet(packet)

            if patient_id not in self.patient_data:
                raise ValueError(f"Invalid patient ID: {patient_id}")

            patient = self.patient_data[patient_id]

            patient["hr_data"].extend(hr_values)
            patient["spo2_data"].extend(spo2_values)
            patient["temp_data"].extend(temp_values)

            patient["hr_curve"].setData(list(patient["hr_data"]))
            patient["spo2_curve"].setData(list(patient["spo2_data"]))
            patient["temp_curve"].setData(list(patient["temp_data"]))

            #active_errors = [code for code in err_codes if code != 0]
            active_errors = [
                index + 1
                for index, flag in enumerate(err_codes)
                if flag == 1
            ]
            self.current_alerts[patient_id] = active_errors

            timestamp = datetime.now().strftime("%I:%M:%S %p")

            if active_errors:
                for code in active_errors:
                    title, subtitle, _ = ERROR_INFO.get(
                        code,
                        (f"UNKNOWN ERROR {code}", "Unknown alert code received.", "#555555")
                    )
                    patient["alert_log"].insertItem(
                        0,
                        f"[{timestamp}] Patient {patient_id}: {title} - {subtitle}"
                    )

            self.refresh_global_alert_banners()

        except Exception as e:
            self.clear_alert_banners()
            self.alert_area.addWidget(
                self.make_banner(
                    "BAD PACKET FORMAT",
                    "Could not parse input string.",
                    "#cc8800"
                )
            )
            print("Error:", e)
            print("Bad packet:", packet)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = VitalsDashboard()
    window.showMaximized()
    sys.exit(app.exec_())