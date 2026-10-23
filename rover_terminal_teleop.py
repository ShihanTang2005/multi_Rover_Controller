#!/usr/bin/env python3
"""ROS 2 Galactic terminal Twist teleop. No pygame, GUI or px4_msgs needed.

Hold W/S/A/D; terminal key repetition renews a short motion lease. A terminal
cannot report key releases: release stops after --key-timeout, not instantly.
Vehicle-side timeout is still required. Space stops; numbers switch and stop.
"""
import argparse
import math
import os
import select
import signal
import sys
import termios
import time
import tty


class Motion:
    def __init__(self, count, speed, turn, timeout):
        self.count, self.speed, self.turn, self.timeout = count, speed, turn, timeout
        self.selected = 0
        self.v = self.w = 0.0
        self.until = 0.0

    def stop(self):
        self.v = self.w = self.until = 0.0

    def key(self, key, now):
        key = key.lower()
        if key in "123456789" and int(key) <= self.count:
            self.stop()
            self.selected = int(key) - 1
        elif key == " ":
            self.stop()
        elif key in ("w", "s", "a", "d"):
            self.v, self.w = {
                "w": (self.speed, 0.0), "s": (-self.speed, 0.0),
                "a": (0.0, self.turn), "d": (0.0, -self.turn),
            }[key]
            self.until = now + self.timeout

    def value(self, now):
        if now >= self.until:
            self.stop()
        return self.v, self.w


def positive(value):
    value = float(value)
    if not math.isfinite(value) or not 0.0 < value <= 1.0:
        raise argparse.ArgumentTypeError("must be finite, greater than 0 and at most 1")
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topics", nargs="+", default=[
        "/roverpi1/cmd_vel", "/roverpi2/cmd_vel", "/roverpi3/cmd_vel"])
    parser.add_argument("--speed", type=positive, default=0.40, help="linear speed in m/s")
    parser.add_argument("--turn", type=positive, default=0.80, help="angular speed in rad/s")
    parser.add_argument("--key-timeout", type=positive, default=0.25,
                        help="seconds since last motion key before zero command")
    args = parser.parse_args()
    if not sys.stdin.isatty():
        parser.error("requires an interactive terminal: use docker exec -it")
    if not 1 <= len(args.topics) <= 9 or len(set(args.topics)) != len(args.topics):
        parser.error("provide 1 to 9 distinct topics")
    if any(not topic.startswith("/") for topic in args.topics):
        parser.error("use absolute topic names starting with /")

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

    rclpy.init(args=[])
    node = rclpy.create_node("rover_terminal_teleop_%d" % os.getpid())
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)
    publishers = [node.create_publisher(Twist, topic, qos) for topic in args.topics]
    motion = Motion(len(args.topics), args.speed, args.turn, args.key_timeout)
    running = True
    old_term = termios.tcgetattr(sys.stdin.fileno())
    old_handlers = {}

    def request_exit(_signum, _frame):
        nonlocal running
        running = False

    def publish(values):
        for publisher, (v, w) in zip(publishers, values):
            msg = Twist()
            msg.linear.x, msg.angular.z = float(v), float(w)
            publisher.publish(msg)

    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_handlers[sig] = signal.signal(sig, request_exit)
        tty.setcbreak(sys.stdin.fileno())
        print("W/S forward/back | A/D rotate | SPACE stop all | 1..9 select | Q/Ctrl+C exit")
        print("Hold a direction key; release stops after %.2fs. No combined directions."
              % args.key_timeout)
        print("Motion allowed only when the selected topic has exactly ONE subscriber.")
        for i, topic in enumerate(args.topics, 1):
            print("%d: %s" % (i, topic))
        print("ROS_DOMAIN_ID=%s" % os.environ.get("ROS_DOMAIN_ID", "0 (unset)"))
        counts = [0] * len(publishers)
        next_discovery = next_status = 0.0
        while running and rclpy.ok():
            start = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            # Discard queued motion keys when switching/stopping. This prevents
            # old key repeats in the same input batch from moving the new rover.
            data = ""
            if select.select([sys.stdin], [], [], 0)[0]:
                data = os.read(sys.stdin.fileno(), 4096).decode("ascii", "ignore")
            if "q" in data.lower() or "\x03" in data:
                running = False
                motion.stop()
            elif " " in data or "\x1b" in data:
                # Arrow keys use escape sequences; deliberately unsupported.
                motion.stop()
            else:
                switches = [k for k in data if k in "123456789" and int(k) <= len(publishers)]
                if switches:
                    motion.key(switches[-1], start)
                else:
                    for key in data:
                        motion.key(key, start)
            if start >= next_discovery:
                counts = [publisher.get_subscription_count() for publisher in publishers]
                next_discovery = start + 0.20
            if counts[motion.selected] != 1:
                motion.stop()
            values = [(0.0, 0.0)] * len(publishers)
            values[motion.selected] = motion.value(start)
            publish(values)
            if start >= next_status:
                v, w = values[motion.selected]
                status = "READY" if counts[motion.selected] == 1 else "BLOCKED"
                sys.stdout.write("\r%s | %s | subscribers=%d | v=%+.2f w=%+.2f       " %
                                 (args.topics[motion.selected], status,
                                  counts[motion.selected], v, w))
                sys.stdout.flush()
                next_status = start + 0.20
            time.sleep(max(0.0, 0.05 - (time.monotonic() - start)))
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_term)
        # Best effort exit zeros. Receiver timeout handles process/network loss.
        try:
            if rclpy.ok():
                for _ in range(6):
                    publish([(0.0, 0.0)] * len(publishers))
                    time.sleep(0.05)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            for sig, handler in old_handlers.items():
                signal.signal(sig, handler)
            print("\nStopped. Terminal restored.")


if __name__ == "__main__":
    main()
