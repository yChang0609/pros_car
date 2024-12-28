# mode_app.py
import urwid
from pros_car_py.mode_manager import VehicleMode, ArmMode, CraneMode, AutoNavMode

# 可自行命名
MODES_REGISTRY = {
    'mode_vehicle':  ("Control Vehicle", VehicleMode),
    'mode_arm':      ("Manual Arm Control", ArmMode),
    'mode_crane':    ("Manual Crane Control", CraneMode),
    'mode_auto_nav': ("Auto Navigation", AutoNavMode),
    # 'mode_auto_arm': ("Automatic Arm Mode", AutoArmMode),
}

class ModeApp:
    def __init__(self, car_controller, arm_controller, custom_control, crane_controller):
        self.car_controller = car_controller
        self.arm_controller = arm_controller
        self.custom_control = custom_control
        self.crane_controller = crane_controller

        self.palette = [('reversed', 'standout', '')]
        self.loop = urwid.MainLoop(None, palette=self.palette)

        self.current_mode = None

    def main(self):
        self.main_menu()
        self.loop.run()

    def main_menu(self):
        """主選單"""
        menu_items = [urwid.Text("Main Menu:")]

        # 動態讀取 MODES_REGISTRY，依序產生選單項目
        for mode_name, (label, mode_class) in MODES_REGISTRY.items():
            button = urwid.Button(label, on_press=lambda _, m=mode_name: self.switch_mode(m))
            menu_items.append(urwid.AttrMap(button, None, focus_map='reversed'))

        # 最後一個選單： Exit
        exit_button = urwid.Button('Exit', on_press=lambda _: self.exit_program())
        menu_items.append(urwid.AttrMap(exit_button, None, focus_map='reversed'))

        menu_list = urwid.ListBox(urwid.SimpleFocusListWalker(menu_items))
        self.loop.widget = menu_list
        # self.loop.unhandled_input = None  # 進入主選單後，先清除 unhandled_input

    def switch_mode(self, mode_name):
        """切換到指定模式"""
        mode_info = MODES_REGISTRY.get(mode_name)
        if not mode_info:
            print(f"Mode '{mode_name}' not found.")
            return

        # 先退出當前模式（若需要）
        if self.current_mode:
            self.current_mode.exit()

        _, ModeClass = mode_info
        # 建立模式實例
        self.current_mode = ModeClass(self)
        # 呼叫進入該模式
        self.current_mode.enter()

    def horizontal_select(self, options, on_select):
        """
        橫向選擇子模式的功能
         - options: 子模式列表 (list)
         - on_select: callback(selected_option) 使用者按下 enter 時要呼叫
        """
        index = [0]

        def render():
            items = [
                urwid.Text(("reversed", f"[{opt}]") if i == index[0] else f" {opt} ")
                for i, opt in enumerate(options)
            ]
            return urwid.Columns(items, dividechars=1)

        def key_handler(key):
            if key == 'left':
                index[0] = (index[0] - 1) % len(options)
                self.loop.widget = urwid.Filler(render(), valign='top')
            elif key == 'right':
                index[0] = (index[0] + 1) % len(options)
                self.loop.widget = urwid.Filler(render(), valign='top')
            elif key == 'enter':
                on_select(options[index[0]])
            elif key == 'q':
                # 返回主選單
                self.main_menu()

        self.loop.widget = urwid.Filler(render(), valign='top')
        self.loop.unhandled_input = key_handler

    def exit_program(self):
        print("Exiting program.")
        raise urwid.ExitMainLoop()

