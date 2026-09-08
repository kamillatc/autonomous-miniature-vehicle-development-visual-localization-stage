import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import yaml
import threading
import sys
from pathlib import Path


OUTPUT_FILE = 'telemetria_rpm.yaml'
RECORD_DURATION_SEC = 30.0


class RpmSaver(Node):
    def __init__(self):
        super().__init__('rpm_saver')

        self.data_log: list[dict] = []
        self._saving = False
        self._lock = threading.Lock()

        self.subscription = self.create_subscription(
            Int32,
            '/RPM',
            self._rpm_callback,
            10,
        )
        self.get_logger().info('RPM subscriber ready on /RPM')

    def start_recording(self):
        with self._lock:
            self.data_log.clear()
            self._saving = True

    def stop_recording(self):
        with self._lock:
            self._saving = False

    def _rpm_callback(self, msg: Int32):
        with self._lock:
            if not self._saving:
                return

            timestamp = self.get_clock().now().nanoseconds * 1e-9

            self.data_log.append({
                'timestamp': float(timestamp),
                'rpm': int(msg.data),
            })

    def save_to_yaml(self, path: str = OUTPUT_FILE) -> int:
        with self._lock:
            snapshot = list(self.data_log)

        output_path = Path(path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        with output_path.open('w', encoding='utf-8') as f:
            yaml.dump(snapshot, f, default_flow_style=False, allow_unicode=True)

        return len(snapshot)


def _spin_for(node: Node, duration_sec: float, stop_event: threading.Event):
    deadline = node.get_clock().now().nanoseconds + int(duration_sec * 1e9)
    while rclpy.ok() and not stop_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.get_clock().now().nanoseconds >= deadline:
            break
    stop_event.set()


def main():
    rclpy.init()
    node = RpmSaver()

    print("Press ENTER to start recording (or Ctrl-C to quit)...")
    try:
        print("Teste 2")
        input()
        print("Teste")
    except (KeyboardInterrupt, EOFError):
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    print(f"Recording RPM for {RECORD_DURATION_SEC:.0f} seconds...")
    node.start_recording()

    stop_event = threading.Event()
    spin_thread = threading.Thread(
        target=_spin_for,
        args=(node, RECORD_DURATION_SEC, stop_event),
        daemon=True,
    )
    spin_thread.start()
    stop_event.wait()

    node.stop_recording()
    spin_thread.join(timeout=2.0)

    count = node.save_to_yaml(OUTPUT_FILE)
    print(f"Done — {count} samples saved to '{OUTPUT_FILE}'")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()