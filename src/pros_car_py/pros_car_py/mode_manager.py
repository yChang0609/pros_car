import urwid
from pros_car_py.base_mode import BaseMode
import threading
import time

class VehicleMode(BaseMode):
    def enter(self):
        text = urwid.Text("Vehicle Mode\nPress 'q' to return to main menu.")
        filler = urwid.Filler(text, valign='top')

        # 切換到這個畫面
        self.app.loop.widget = filler
        # 設定鍵盤事件處理器
        self.app.loop.unhandled_input = self.handle_input

    def handle_input(self, key):
        if key == 'q':
            self.app.main_menu()
        else:
            # 呼叫 car_controller 做實際車輛控制
            self.app.car_controller.manual_control(key)

class ArmMode(BaseMode):
    # 在這裡定義所有子模式
    submodes = ["0", "1", "2", "3", "4"]

    def enter(self):
        """進入手臂模式 -> 顯示橫向選單選擇子模式"""
        self.app.horizontal_select(
            self.submodes,
            self.handle_submode_select  # 使用者按 enter 時呼叫
        )

    def handle_submode_select(self, submode):
        def on_key(key):
            # 子模式的具體邏輯
            self.app.arm_controller.manual_control(int(submode), key)
            
        # on_quit 不寫，表示按 'q' 時就回到自己 (enter)，也就是回到水平選單
        self.show_submode_screen(
            message=f"Arm Mode: Submode {submode}\nPress 'q' to go back.",
            on_key=on_key
        )


class CraneMode(BaseMode):
    submodes = ["0", "1", "2", "3", "4", "5", "6", "99"]

    def enter(self):
        self.app.horizontal_select(
            self.submodes,
            self.handle_submode_select
        )

    def handle_submode_select(self, submode):
        def on_key(key):
            # 子模式的具體邏輯
            self.app.crane_controller.manual_control(int(submode), key)


        self.show_submode_screen(
            message=f"Crane Mode: Submode {submode}\nPress 'q' to go back.",
            on_key=on_key
        )

class AutoNavMode(BaseMode):
    submodes = ["manual_auto_nav", "target_auto_nav"]

    def enter(self):
        self.app.horizontal_select(
            self.submodes,
            self.handle_submode_select
        )

    def handle_submode_select(self, submode):
        # 像 CraneMode 一樣，定義 on_key
        def on_key(key):
            self.app.car_controller.auto_control(submode, key)
            if key == 'q':
                self.app.car_controller.auto_control(submode, key=key)

        self.show_submode_screen(
            message=f"AutoNav Mode: Submode {submode}\nPress 'q' to go back.",
            on_key=on_key
        )

class AutoArmMode(BaseMode):
    submodes = ["Arm Mode 1", "Arm Mode 2", "Arm Mode 3"]

    def enter(self):
        self.app.horizontal_select(
            self.submodes,
            self.handle_submode_select
        )

    def handle_submode_select(self, submode):
        def on_key(key):
            # 子模式的具體邏輯
            self.app.crane_controller.manual_control(int(submode), key)


        self.show_submode_screen(
            message=f"Crane Mode: Submode {submode}\nPress 'q' to go back.",
            on_key=on_key
        )

