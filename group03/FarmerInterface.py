import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import tkinter as tk
from playsound import playsound
from sensor_msgs.msg import BatteryState
import threading
import os
from ament_index_python.packages import get_package_share_directory


class FarmerCommandNode(Node):
    def __init__(self):
        super().__init__("farmer_command_node")
        self.publisher_ = self.create_publisher(String, "farmer_cmd", 10)
        self.subscription = self.create_subscription(
            String, "fsm_state", self.fsm_callback, 10
        )
        self.battery_sub = self.create_subscription(
            BatteryState, "/io/power/power_watcher", self.battery_callback, 10
        )
        self.current_state = "Waiting for Command"
        self.last_state = ""
        self.battery_level = 100.0
        self.sounds_enabled = True
        self.get_logger().info("GUI ready. Click buttons to send commands.")
        # Dynamically get the path to your ROS 2 package
        package_share = get_package_share_directory("group03")
        self.sound_dir = os.path.join(package_share, "sounds")

        # self.sound_dir = '/home/jasper/mdp_sim/src/group03/group03/sounds'
        self.sound_map = {
            "WAIT_FOR_COMMAND": os.path.join(self.sound_dir, "WAITING_FOR_COMMAND.wav"),
            "STOPPED": os.path.join(self.sound_dir, "EMERGENCY_STOP.wav"),
            "RETURNING_HOME": os.path.join(self.sound_dir, "RETURNING_HOME.wav"),
            "PROCESS_ORDER": os.path.join(self.sound_dir, "PROCESS_ORDER.wav"),
            # "NAV_TO_TREE": os.path.join(self.sound_dir, "mixkit-confirmation-tone-2867.wav"),
            "OBSTACLE_AVOIDANCE": os.path.join(
                self.sound_dir, "mixkit-confirmation-tone-2867.wav"
            ),
            # "DETECTING_APPLES": os.path.join(self.sound_dir, "mixkit-confirmation-tone-2867.wav"),
            # "PICKING_APPLE": os.path.join(self.sound_dir, "mixkit-confirmation-tone-2867.wav"),
            # "NAV_TO_BASKET": os.path.join(self.sound_dir, "mixkit-confirmation-tone-2867.wav"),
            # "PLACING_APPLE": os.path.join(self.sound_dir, "mixkit-confirmation-tone-2867.wav"),
            "COMMUNICATING": os.path.join(
                self.sound_dir, "mixkit-confirmation-tone-2867.wav"
            ),
            "EXCEPTION": os.path.join(self.sound_dir, "fail-234710.mp3"),
        }

        self.build_gui()

    def build_gui(self):
        self.root = tk.Tk()
        self.root.title("👨‍🌾 Farmer Command Interface")
        self.root.configure(bg="#f0f0f0")

        tk.Label(
            self.root,
            text="🍏 Farmer Control Panel",
            font=("Helvetica", 24, "bold"),
            bg="#f0f0f0",
            fg="#333",
        ).pack(pady=20)

        btn_font = ("Helvetica", 18, "bold")
        btn_width = 25
        btn_height = 2
        pad = 10

        tk.Button(
            self.root,
            text="🟥 EMERGENCY STOP",
            bg="#d32f2f",
            fg="white",
            font=btn_font,
            width=btn_width,
            height=btn_height,
            command=lambda: self.send_command("stop"),
        ).pack(pady=pad)

        tk.Button(
            self.root,
            text="🔁 Return Home",
            bg="#f57c00",
            fg="white",
            font=btn_font,
            width=btn_width,
            height=btn_height,
            command=lambda: self.send_command("return_to_start"),
        ).pack(pady=pad)

        tk.Button(
            self.root,
            text="📦 Order Request",
            bg="#388e3c",
            fg="white",
            font=btn_font,
            width=btn_width,
            height=btn_height,
            command=lambda: self.send_command("order_request"),
        ).pack(pady=pad)

        self.status_label = tk.Label(
            self.root,
            text="Robot State: Waiting for Command",
            font=("Helvetica", 18),
            bg="#f0f0f0",
            fg="#1565c0",
        )
        self.status_label.pack(pady=10)

        self.battery_label = tk.Label(
            self.root,
            text="🔋 Battery: 100.0%",
            font=("Helvetica", 14),
            bg="#f0f0f0",
            fg="#333",
        )
        self.battery_label.pack(pady=5)

        self.sound_toggle_button = tk.Button(
            self.root,
            text="🔊 Sounds: ON",
            bg="#9e9e9e",
            fg="white",
            font=("Helvetica", 12, "bold"),
            width=20,
            command=self.toggle_sounds,
        )
        self.sound_toggle_button.pack(pady=10)

        self.root.after(1, self.tk_loop)
        self.root.mainloop()

    def toggle_sounds(self):
        self.sounds_enabled = not self.sounds_enabled
        state_text = "ON" if self.sounds_enabled else "OFF"
        self.sound_toggle_button.config(text=f"🔊 Sounds: {state_text}")
        self.get_logger().info(
            f"Sound playback {'enabled' if self.sounds_enabled else 'disabled'}."
        )

    def tk_loop(self):
        rclpy.spin_once(self, timeout_sec=0)
        self.status_label.config(text=f"Robot State: {self.current_state}")
        self.battery_label.config(text=f"🔋 Battery: {self.battery_level:.1f}%")
        self.root.after(1, self.tk_loop)

    def send_command(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published command: {cmd}")

    def fsm_callback(self, msg: String):
        new_state = msg.data
        if new_state != self.current_state:
            self.last_state = self.current_state
            self.current_state = new_state
            self.get_logger().info(f"FSM state updated: {self.current_state}")
            self.play_state_sound(new_state)

    def battery_callback(self, msg: BatteryState):
        if msg.percentage >= 0.0:
            self.battery_level = msg.percentage * 100.0
        else:
            self.battery_level = 0.0

    def play_state_sound(self, state_name):
        if not self.sounds_enabled:
            return
        sound_file = self.sound_map.get(state_name)
        if sound_file and os.path.exists(sound_file):
            threading.Thread(target=playsound, args=(sound_file,), daemon=True).start()
        else:
            self.get_logger().warn(f"No sound file found for state: {state_name}")


def main(args=None):
    rclpy.init(args=args)
    node = FarmerCommandNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
