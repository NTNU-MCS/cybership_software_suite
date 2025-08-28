#!/usr/bin/env python3

"""
PyQt GUI to control topic_tools multiplexers for force or velocity commands.

Features
- Enter/select a vehicle namespace (e.g., /enterprise)
- Discover mux services and list input topics via MuxList
- Switch the active source using MuxSelect
"""

import os
import sys
from typing import Optional, List

from PyQt5 import QtWidgets, QtCore

import rclpy
from rclpy.node import Node

try:
    from topic_tools_interfaces.srv import MuxList, MuxSelect
    from std_srvs.srv import Empty
except Exception:
    print("Missing dependency topic_tools_interfaces. Please build/install it.")
    raise


DEFAULT_MUX_NAME = os.environ.get("MUX_NAME", "force_mux")


class ForceMuxClient(Node):
    def __init__(self, namespace: str, mux_name: str = DEFAULT_MUX_NAME) -> None:
        super().__init__("force_mux_client", namespace=namespace)
        self.mux_name = mux_name
        self._list_cli = self.create_client(MuxList, f"{mux_name}/list")
        self._select_cli = self.create_client(MuxSelect, f"{mux_name}/select")

    def discover_mux_bases(self) -> List[str]:
        bases: List[str] = []
        ns_prefix = self.get_namespace().rstrip("/")
        for name, types in self.get_service_names_and_types():
            flat_types: List[str] = []
            for t in types:
                if isinstance(t, str):
                    flat_types.append(t)
                else:
                    flat_types.extend(t)
            if not any("topic_tools_interfaces/srv/MuxList" in t for t in flat_types):
                continue
            if ns_prefix and not name.startswith(ns_prefix + "/"):
                continue
            if not name.endswith("/list"):
                continue
            base = name.split("/")[-2]
            bases.append(base)
        return sorted(set(bases))

    def wait_for_services(self, timeout_sec: float = 2.0) -> bool:
        ok1 = self._list_cli.wait_for_service(timeout_sec=timeout_sec)
        ok2 = self._select_cli.wait_for_service(timeout_sec=timeout_sec)
        return bool(ok1 and ok2)

    def list_topics(self):
        req = MuxList.Request()
        return self._list_cli.call_async(req)

    def select_topic(self, topic: str):
        req = MuxSelect.Request()
        req.topic = topic
        return self._select_cli.call_async(req)


class MuxPanel(QtWidgets.QGroupBox):
    def __init__(self, kind: str, get_namespace, ensure_rcl) -> None:
        title = "Force Mux" if kind == "force" else "Velocity Mux"
        super().__init__(title)
        self.kind = kind  # 'force' or 'velocity'
        self._get_namespace = get_namespace
        self._ensure_rcl = ensure_rcl
        self._node = None  # type: Optional[ForceMuxClient]

        self.topic_combo = QtWidgets.QComboBox()
        self.topic_combo.setEditable(False)
        self.find_btn = QtWidgets.QPushButton("Find")
        self.refresh_btn = QtWidgets.QPushButton("Refresh")
        self.select_btn = QtWidgets.QPushButton("Select")
        self.status_lbl = QtWidgets.QLabel("Idle")

        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.topic_combo, 1)
        row.addWidget(self.find_btn)
        row.addWidget(self.refresh_btn)
        row.addWidget(self.select_btn)

        lay = QtWidgets.QVBoxLayout(self)
        lay.addLayout(row)
        lay.addWidget(self.status_lbl)

        self.find_btn.clicked.connect(self.on_find)
        self.refresh_btn.clicked.connect(self.on_refresh)
        self.select_btn.clicked.connect(self.on_select)

    def _prefix(self) -> str:
        return "force_mux" if self.kind == "force" else "velocity_mux"

    def _desired_default_mux_name(self) -> str:
        # Allow per-kind override, else global, else default
        per_kind = os.environ.get("FORCE_MUX_NAME" if self.kind == "force" else "VELOCITY_MUX_NAME")
        if per_kind:
            return per_kind
        env_override = os.environ.get("MUX_NAME")
        if env_override:
            return env_override
        return self._prefix()

    def _namespace(self) -> str:
        ns = (self._get_namespace() or "").strip()
        if ns and not ns.startswith("/"):
            ns = "/" + ns
        return ns

    def _recreate_node(self, mux_name: Optional[str] = None) -> Optional[ForceMuxClient]:
        ns = self._namespace()
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None
        self._ensure_rcl()
        try:
            self._node = ForceMuxClient(namespace=ns, mux_name=mux_name or self._desired_default_mux_name())
        except Exception as e:
            self.status_lbl.setText(f"Node error: {e}")
            self._node = None
        return self._node

    def spin_once(self) -> None:
        if self._node is not None and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.0)

    def on_find(self) -> None:
        node = self._recreate_node()
        if node is None:
            return
        self.status_lbl.setText("Discovering mux…")
        bases = node.discover_mux_bases()
        if not bases:
            self.status_lbl.setText("No mux found")
            return
        preferred = [b for b in bases if b.startswith(self._prefix())]
        chosen = preferred[0] if preferred else bases[0]
        # Bind to chosen mux
        self._recreate_node(mux_name=chosen)
        self.status_lbl.setText(f"Using '{chosen}'")

    def on_refresh(self) -> None:
        node = self._recreate_node()
        if node is None:
            return
        self.status_lbl.setText("Waiting for services…")
        if not node.wait_for_services(timeout_sec=2.0):
            bases = node.discover_mux_bases()
            if not bases:
                self.status_lbl.setText("Mux services unavailable")
                return
            chosen = (next((b for b in bases if b.startswith(self._prefix())), None) or bases[0])
            node = self._recreate_node(mux_name=chosen)
            if node is None or not node.wait_for_services(timeout_sec=2.0):
                self.status_lbl.setText("Mux services unavailable")
                return
            self.status_lbl.setText(f"Connected '{chosen}'")

        self.status_lbl.setText("Listing…")
        fut = node.list_topics()

        def on_done():
            if fut.cancelled() or fut.exception() is not None:
                self.status_lbl.setText(f"List failed: {fut.exception()}")
                return
            resp = fut.result()
            topics = list(getattr(resp, "topics", []))
            self.topic_combo.clear()
            if topics:
                self.topic_combo.addItems(topics)
                self.status_lbl.setText(f"{len(topics)} topic(s)")
            else:
                self.status_lbl.setText("No topics in mux")

        fut.add_done_callback(lambda _: QtCore.QTimer.singleShot(0, on_done))

    def on_select(self) -> None:
        if self._node is None:
            self.status_lbl.setText("Find/Refresh first")
            return
        topic = self.topic_combo.currentText().strip()
        if not topic:
            self.status_lbl.setText("Pick a topic")
            return
        self.status_lbl.setText(f"Selecting {topic}…")
        fut = self._node.select_topic(topic)

        def on_done():
            if fut.cancelled() or fut.exception() is not None:
                self.status_lbl.setText(f"Select failed: {fut.exception()}")
                return
            resp = fut.result()
            ok = bool(getattr(resp, "success", False))
            self.status_lbl.setText("Selected" if ok else "Select rejected")

        fut.add_done_callback(lambda _: QtCore.QTimer.singleShot(0, on_done))


class BothMuxGUI(QtWidgets.QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Mux Controller (Force + Velocity)")
        self.setMinimumWidth(640)

        # rcl state
        self._rcl_inited = False

        # Top controls
        self.ns_edit = QtWidgets.QLineEdit()
        self.ns_edit.setPlaceholderText("/vehicle_namespace (e.g., /enterprise)")
        self.ns_edit.setText(os.environ.get("VEHICLE_NS", ""))

        form = QtWidgets.QFormLayout()
        form.addRow("Namespace", self.ns_edit)

        # Panels (vertical stack)
        self.force_panel = MuxPanel("force", get_namespace=lambda: self.ns_edit.text(), ensure_rcl=self._ensure_rcl)
        self.velocity_panel = MuxPanel("velocity", get_namespace=lambda: self.ns_edit.text(), ensure_rcl=self._ensure_rcl)

        panels = QtWidgets.QVBoxLayout()
        panels.addWidget(self.force_panel)
        panels.addWidget(self.velocity_panel)

        # Thruster control
        thruster_box = QtWidgets.QGroupBox("Thruster")
        self.thr_activate_btn = QtWidgets.QPushButton("Activate")
        self.thr_disable_btn = QtWidgets.QPushButton("Deactivate")
        self.thr_status_lbl = QtWidgets.QLabel("Idle")
        thr_buttons = QtWidgets.QHBoxLayout()
        thr_buttons.addWidget(self.thr_activate_btn)
        thr_buttons.addWidget(self.thr_disable_btn)
        thr_lay = QtWidgets.QVBoxLayout(thruster_box)
        thr_lay.addLayout(thr_buttons)
        thr_lay.addWidget(self.thr_status_lbl)

        root = QtWidgets.QVBoxLayout(self)
        root.addLayout(form)
        root.addWidget(thruster_box)
        root.addLayout(panels)

        # Timer to spin both nodes
        self._spin_timer = QtCore.QTimer(self)
        self._spin_timer.timeout.connect(self._spin_once)
        self._spin_timer.start(20)

        # Thruster node/state
        self._thruster_node = None
        self._thruster_ns = None
        self.thr_activate_btn.clicked.connect(self.on_thr_activate)
        self.thr_disable_btn.clicked.connect(self.on_thr_disable)

    def _ensure_rcl(self) -> None:
        if not self._rcl_inited:
            rclpy.init(args=None)
            self._rcl_inited = True

    def _spin_once(self) -> None:
        self.force_panel.spin_once()
        self.velocity_panel.spin_once()
        # Spin thruster node too
        if self._thruster_node is not None and rclpy.ok():
            rclpy.spin_once(self._thruster_node, timeout_sec=0.0)

    def _namespace(self) -> str:
        ns = (self.ns_edit.text() or "").strip()
        if ns and not ns.startswith("/"):
            ns = "/" + ns
        return ns

    def _ensure_thruster_node(self):
        # Create once per namespace; reuse to avoid handle invalidation
        ns = self._namespace()
        if self._thruster_node is None or self._thruster_ns != ns:
            # Dispose old (if any) before replacing
            if self._thruster_node is not None:
                try:
                    self._thruster_node.destroy_node()
                except Exception:
                    pass
                self._thruster_node = None
            self._ensure_rcl()
            self._thruster_node = rclpy.create_node("thruster_client", namespace=ns)
            self._thruster_ns = ns
        return self._thruster_node

    def _discover_thruster_services(self, name_candidates, node) -> list:
        found = []
        empty_type_names = {"std_srvs/srv/Empty", "std_srvs/srv/Empty.srv"}
        services = node.get_service_names_and_types()
        ns = self._namespace() or "/"
        ns_prefix = ns.rstrip("/") + "/" if ns != "/" else "/"
        for cand in name_candidates:
            for name, types in services:
                flat_types = []
                for t in types:
                    flat_types.append(t if isinstance(t, str) else t[0])
                if not any(t in empty_type_names for t in flat_types):
                    continue
                if not name.startswith(ns_prefix):
                    continue
                if name.endswith("/" + cand):
                    found.append(name)
        # Unique preserve order
        seen = set()
        ordered = []
        for n in found:
            if n not in seen:
                seen.add(n)
                ordered.append(n)
        return ordered

    def _call_empty_service(self, name_candidates) -> bool:
        node = self._ensure_thruster_node()
        # Try discovered fully-qualified matches first (using same node)
        fqns = self._discover_thruster_services(name_candidates, node)
        try_list = fqns + name_candidates
        for svc_name in try_list:
            cli = node.create_client(Empty, svc_name)
            if not cli.wait_for_service(timeout_sec=1.5):
                continue
            fut = cli.call_async(Empty.Request())
            for _ in range(30):
                rclpy.spin_once(node, timeout_sec=0.02)
                if fut.done():
                    break
            if fut.done() and fut.exception() is None:
                return True
        return False

    def on_thr_activate(self) -> None:
        self.thr_status_lbl.setText("Activating…")
        ok = self._call_empty_service(["thruster/activate", "thruster/enable"])  # fallback to enable
        self.thr_status_lbl.setText("Activated" if ok else "Activate failed")

    def on_thr_disable(self) -> None:
        self.thr_status_lbl.setText("Disabling…")
        ok = self._call_empty_service(["thruster/disable"])  # strict name per request
        self.thr_status_lbl.setText("Disabled" if ok else "Disable failed")


def main() -> None:
    app = QtWidgets.QApplication(sys.argv)
    gui = BothMuxGUI()
    gui.show()
    rc = app.exec_()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
