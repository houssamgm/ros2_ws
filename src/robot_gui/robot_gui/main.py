#!/usr/bin/env python3

import os
import sys
import rclpy
from threading import Thread

from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QLabel,
    QVBoxLayout, QHBoxLayout, QFrame, QGridLayout
)

from PyQt5.QtCore import Qt

from robot_gui.ros_interface import ROSInterface


class RobotGUI(QWidget):

    def __init__(self):
        super().__init__()

        self.setWindowTitle("Industrial WOA Robot System (MUX)")
        self.setGeometry(100, 100, 1200, 800)

        # ================= ROS INIT =================
        if not rclpy.ok():
            rclpy.init()

        self.ros = ROSInterface()
        self.ros.state_callback_fn = self.update_state

        self.ros_thread = Thread(
            target=rclpy.spin,
            args=(self.ros,),
            daemon=True
        )
        self.ros_thread.start()

        # ================= UI =================
        self.build_ui()
        self.load_styles()

    # =========================================================
    # STATE DISPLAY
    # =========================================================
    def update_state(self, state):
        self.robot_state.setText(f"Robot State: {state}")

    # =========================================================
    # UI BUILD
    # =========================================================
    def build_ui(self):

        main_layout = QHBoxLayout()

        # ================= LEFT PANEL =================
        left_panel = QFrame()
        left_panel.setObjectName("leftPanel")   # QSS HOOK
        left_layout = QVBoxLayout(left_panel)

        # ---- MODE SECTION ----
        title_modes = QLabel("ROBOT MODES")
        title_modes.setObjectName("sectionTitle")

        self.woa_btn = QPushButton("START WOA FOLLOW")
        self.stop_woa_btn = QPushButton("STOP WOA")
        self.nav_stop_btn = QPushButton("STOP NAV MODE (FREE ROBOT)")

        self.woa_btn.setMinimumHeight(60)
        self.stop_woa_btn.setMinimumHeight(60)
        self.nav_stop_btn.setMinimumHeight(60)

        # ---- GOALS SECTION ----
        title_goals = QLabel("NAVIGATION GOALS")
        title_goals.setObjectName("sectionTitle")

        dest_grid = QGridLayout()
        self.goal_buttons = []

        for i, name in enumerate(self.ros.locations.keys()):

            btn = QPushButton(name.upper())
            btn.setMinimumHeight(70)

            btn.clicked.connect(lambda _, n=name: self.ros.send_goal(n))

            dest_grid.addWidget(btn, i // 2, i % 2)
            self.goal_buttons.append(btn)

        # ---- EXIT ----
        self.exit_btn = QPushButton("EXIT")

        # ================= RIGHT PANEL =================
        right_panel = QFrame()
        right_panel.setObjectName("rightPanel")  # QSS HOOK
        right_layout = QVBoxLayout(right_panel)

        status_title = QLabel("SYSTEM STATUS")
        status_title.setObjectName("sectionTitle")

        self.robot_state = QLabel("Robot State: IDLE")
        self.robot_state.setObjectName("statusLabel")

        self.nav_state = QLabel("Nav2: READY")
        self.nav_state.setObjectName("statusLabel")

        self.woa_state = QLabel("WOA: READY")
        self.woa_state.setObjectName("statusLabel")

        right_layout.addWidget(status_title)
        right_layout.addWidget(self.robot_state)
        right_layout.addWidget(self.nav_state)
        right_layout.addWidget(self.woa_state)

        right_layout.addStretch()

        # ================= EMERGENCY BUTTON =================
        self.estop_btn = QPushButton("EMERGENCY STOP")
        self.estop_btn.setObjectName("dangerButtonRound")  # QSS HOOK
        self.estop_btn.setFixedSize(160, 160)

        right_layout.addWidget(self.estop_btn, alignment=Qt.AlignCenter)

        # ================= LAYOUT =================
        left_layout.addWidget(title_modes)
        left_layout.addWidget(self.woa_btn)
        left_layout.addWidget(self.stop_woa_btn)
        left_layout.addWidget(self.nav_stop_btn)

        left_layout.addWidget(title_goals)
        left_layout.addLayout(dest_grid)
        left_layout.addWidget(self.exit_btn)

        main_layout.addWidget(left_panel, 3)
        main_layout.addWidget(right_panel, 2)

        self.setLayout(main_layout)

        # ================= CONNECTIONS (UNCHANGED LOGIC) =================
        self.woa_btn.clicked.connect(self.ros.start_woa_following)
        self.stop_woa_btn.clicked.connect(self.ros.stop_woa_following)
        self.nav_stop_btn.clicked.connect(self.ros.emergency_stop)
        self.estop_btn.clicked.connect(self.ros.emergency_stop)
        self.exit_btn.clicked.connect(self.close)

    # =========================================================
    # STYLE LOADER
    # =========================================================
    def load_styles(self):
        path = os.path.join(os.path.dirname(__file__), "styles.qss")
        if os.path.exists(path):
            with open(path, "r") as f:
                self.setStyleSheet(f.read())

    # =========================================================
    # CLEAN EXIT
    # =========================================================
    def closeEvent(self, event):
        try:
            self.ros.stop_robot()
            self.ros.destroy_node()
            rclpy.shutdown()
        except:
            pass
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = RobotGUI()
    window.show()
    sys.exit(app.exec_())