#!/usr/bin/env python3

"""
WebSocket server for the web-based ROS 2 control interface.

Provides a simple JSON-based WebSocket protocol to:
- List and select force/velocity mux topics
- Activate/deactivate thrusters
- Auto-discover available services

Run: ros2 run cybership_utilities web_interface_server.py
"""

import asyncio
import json
import logging
from typing import Dict, List, Optional, Set
import websockets
from websockets.asyncio.server import serve

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

try:
    from topic_tools_interfaces.srv import MuxList, MuxSelect
    from std_srvs.srv import Empty
    from lifecycle_msgs.srv import ChangeState, GetState
    from lifecycle_msgs.msg import Transition, State
except ImportError:
    print("Missing dependencies: topic_tools_interfaces, std_srvs, lifecycle_msgs")
    raise


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class ROS2Bridge(Node):
    """ROS 2 node that handles service calls."""

    def __init__(self):
        super().__init__("web_interface_bridge", namespace="")

        # Declare parameter for WebSocket port
        self.declare_parameter("ws_port", 8765)

        self._service_clients: Dict[str, Dict] = {}  # namespace -> {list_cli, select_cli, ...}
        self.service_cache: List[Dict] = []
        self.last_scan_time = 0.0

    def get_or_create_clients(self, namespace: str, mux_name: str):
        """Get or create service clients for a given namespace and mux name."""
        key = f"{namespace}/{mux_name}"

        if key not in self._service_clients:
            ns_prefix = namespace.rstrip("/")
            service_base = f"{ns_prefix}/{mux_name}" if namespace else f"/{mux_name}"

            self._service_clients[key] = {
                "list": self.create_client(MuxList, f"{service_base}/list"),
                "select": self.create_client(MuxSelect, f"{service_base}/select"),
            }
            logger.info(f"Created clients for {service_base}")

        return self._service_clients[key]

    def get_or_create_empty_client(self, service_name: str):
        """Get or create an Empty service client."""
        key = f"empty:{service_name}"

        if key not in self._service_clients:
            self._service_clients[key] = {
                "empty": self.create_client(Empty, service_name)
            }
            logger.info(f"Created Empty client for {service_name}")

        return self._service_clients[key]["empty"]

    @staticmethod
    def _normalize_namespace(namespace: str) -> str:
        ns = (namespace or "").strip()
        if not ns:
            return ""
        ns = ns.strip("/")
        return f"/{ns}" if ns else ""

    def get_or_create_lifecycle_clients(self, namespace: str, node_name: str):
        """Get or create lifecycle service clients for a node."""
        ns = self._normalize_namespace(namespace)
        key = f"lifecycle:{ns}/{node_name}"

        if key not in self._service_clients:
            service_base = f"{ns}/{node_name}" if ns else f"/{node_name}"
            self._service_clients[key] = {
                "get_state": self.create_client(GetState, f"{service_base}/get_state"),
                "change_state": self.create_client(ChangeState, f"{service_base}/change_state"),
            }
            logger.info(f"Created lifecycle clients for {service_base}")

        return self._service_clients[key]

    async def call_lifecycle_get_state(self, namespace: str, node_name: str) -> Dict:
        clients = self.get_or_create_lifecycle_clients(namespace, node_name)
        cli = clients["get_state"]

        if not cli.wait_for_service(timeout_sec=2.0):
            raise Exception(f"Service not available: {cli.srv_name}")

        req = GetState.Request()
        future = cli.call_async(req)

        timeout = 5.0
        start = self.get_clock().now()
        while not future.done():
            await asyncio.sleep(0.01)
            if (self.get_clock().now() - start).nanoseconds / 1e9 > timeout:
                raise Exception("Service call timeout")

        resp = future.result()
        if not resp:
            return {"id": -1, "label": ""}
        return {"id": int(resp.current_state.id), "label": str(resp.current_state.label)}

    async def call_lifecycle_change_state(
        self,
        namespace: str,
        node_name: str,
        transition_id: int,
    ) -> bool:
        clients = self.get_or_create_lifecycle_clients(namespace, node_name)
        cli = clients["change_state"]

        if not cli.wait_for_service(timeout_sec=2.0):
            raise Exception(f"Service not available: {cli.srv_name}")

        req = ChangeState.Request()
        req.transition = Transition()
        req.transition.id = int(transition_id)
        future = cli.call_async(req)

        timeout = 5.0
        start = self.get_clock().now()
        while not future.done():
            await asyncio.sleep(0.01)
            if (self.get_clock().now() - start).nanoseconds / 1e9 > timeout:
                raise Exception("Service call timeout")

        resp = future.result()
        return bool(resp.success) if resp else False

    def resolve_allocator_node_name(self, namespace: str, explicit_node_name: Optional[str] = None) -> str:
        if explicit_node_name:
            return explicit_node_name

        # Guess from namespace (/voyager -> voyager_thrust_allocator)
        ns = self._normalize_namespace(namespace).strip("/")
        model = ns.split("/")[-1] if ns else ""
        if model in ("voyager", "enterprise", "drillship"):
            return f"{model}_thrust_allocator"
        return "thrust_allocator"

    def find_allocator_lifecycle_node(self, namespace: str, node_candidates: Optional[List[str]] = None) -> str:
        """Find an allocator lifecycle node in a namespace by inspecting available services."""
        ns = self._normalize_namespace(namespace)
        if not self.service_cache:
            self.scan_services()

        if node_candidates:
            for node_name in node_candidates:
                svc_name = f"{ns}/{node_name}/change_state" if ns else f"/{node_name}/change_state"
                for svc in self.service_cache:
                    if svc["name"] == svc_name and "lifecycle_msgs/srv/ChangeState" in svc["type"]:
                        return node_name

        # Fallback: pick the first ChangeState service under the namespace.
        for svc in self.service_cache:
            if "lifecycle_msgs/srv/ChangeState" not in svc["type"]:
                continue
            if ns and not svc["name"].startswith(f"{ns}/"):
                continue
            parts = svc["name"].strip("/").split("/")
            if len(parts) >= 2 and parts[-1] == "change_state":
                return parts[-2]

        raise Exception("No lifecycle ChangeState service found for thrust allocator")

    def scan_services(self) -> List[Dict]:
        """Scan and return available services."""
        services = []
        node_name = self.get_name()

        for name, types in self.get_service_names_and_types():
            # Skip the node's own internal services
            if name.startswith(f"/{node_name}/"):
                continue

            for type_name in types:
                services.append({"name": name, "type": type_name})

        self.service_cache = services
        self.last_scan_time = self.get_clock().now().nanoseconds / 1e9
        return services

    async def call_mux_list(self, namespace: str, mux_name: str) -> List[str]:
        """Call MuxList service."""
        clients = self.get_or_create_clients(namespace, mux_name)
        list_cli = clients["list"]

        if not list_cli.wait_for_service(timeout_sec=2.0):
            raise Exception(f"Service not available: {list_cli.srv_name}")

        req = MuxList.Request()
        future = list_cli.call_async(req)

        # Wait for response
        timeout = 5.0
        start = self.get_clock().now()
        while not future.done():
            await asyncio.sleep(0.01)
            if (self.get_clock().now() - start).nanoseconds / 1e9 > timeout:
                raise Exception("Service call timeout")

        resp = future.result()
        return list(resp.topics) if resp else []

    async def call_mux_select(self, namespace: str, mux_name: str, topic: str) -> Dict:
        """Call MuxSelect service."""
        clients = self.get_or_create_clients(namespace, mux_name)
        select_cli = clients["select"]

        if not select_cli.wait_for_service(timeout_sec=2.0):
            raise Exception(f"Service not available: {select_cli.srv_name}")

        req = MuxSelect.Request()
        req.topic = topic
        future = select_cli.call_async(req)

        # Wait for response
        timeout = 5.0
        start = self.get_clock().now()
        while not future.done():
            await asyncio.sleep(0.01)
            if (self.get_clock().now() - start).nanoseconds / 1e9 > timeout:
                raise Exception("Service call timeout")

        resp = future.result()
        return {
            "prev_topic": resp.prev_topic if resp else "",
            "success": True
        }

    async def call_empty_service(self, service_name: str) -> bool:
        """Call an Empty service."""
        cli = self.get_or_create_empty_client(service_name)

        if not cli.wait_for_service(timeout_sec=2.0):
            raise Exception(f"Service not available: {service_name}")

        req = Empty.Request()
        future = cli.call_async(req)

        # Wait for response
        timeout = 5.0
        start = self.get_clock().now()
        while not future.done():
            await asyncio.sleep(0.01)
            if (self.get_clock().now() - start).nanoseconds / 1e9 > timeout:
                raise Exception("Service call timeout")

        future.result()  # Just check it completed
        return True

    def find_service_flexible(self, candidates: List[str], namespace: str, type_pattern: str) -> Optional[str]:
        """Find a service by trying exact matches and suffix matches."""
        ns_prefix = namespace.rstrip("/") if namespace else ""

        # First pass: exact matches
        for cand in candidates:
            service_name = f"{ns_prefix}/{cand}".lstrip("/") if ns_prefix else cand
            for svc in self.service_cache:
                if svc["name"] == f"/{service_name}" and type_pattern in svc["type"]:
                    return svc["name"]

        # Second pass: suffix matches under namespace
        if ns_prefix:
            for cand in candidates:
                for svc in self.service_cache:
                    if type_pattern not in svc["type"]:
                        continue
                    if not svc["name"].startswith(f"/{ns_prefix}/"):
                        continue
                    if svc["name"].endswith(cand):
                        return svc["name"]

        return None


class WebSocketServer:
    """WebSocket server that handles client connections."""

    def __init__(self, ros_bridge: ROS2Bridge, host: str = "0.0.0.0", port: int = 8765):
        self.ros_bridge = ros_bridge
        self.host = host
        self.port = port
        self.clients: Set = set()

    async def handler(self, websocket):
        """Handle a WebSocket connection."""
        client_id = id(websocket)
        self.clients.add(websocket)
        logger.info(f"Client connected: {client_id}")

        try:
            # Send initial server info
            await websocket.send(json.dumps({
                "type": "serverInfo",
                "name": "ROS 2 Web Interface Bridge",
                "version": "1.0"
            }))

            # Scan and send services
            services = self.ros_bridge.scan_services()
            await websocket.send(json.dumps({
                "type": "services",
                "services": services
            }))

            # Handle incoming messages
            async for message in websocket:
                try:
                    data = json.loads(message)
                    response = await self.handle_message(data)
                    await websocket.send(json.dumps(response))
                except json.JSONDecodeError:
                    await websocket.send(json.dumps({
                        "type": "error",
                        "message": "Invalid JSON"
                    }))
                except Exception as e:
                    logger.error(f"Error handling message: {e}", exc_info=True)
                    await websocket.send(json.dumps({
                        "type": "error",
                        "message": str(e)
                    }))

        except websockets.exceptions.ConnectionClosed:
            logger.info(f"Client disconnected: {client_id}")
        finally:
            self.clients.discard(websocket)

    async def handle_message(self, data: Dict) -> Dict:
        """Handle a message from a client."""
        msg_type = data.get("type")

        if msg_type == "mux_list":
            namespace = data.get("namespace", "")
            mux_name = data.get("mux_name", "force_mux")
            topics = await self.ros_bridge.call_mux_list(namespace, mux_name)
            return {
                "type": "mux_list_response",
                "topics": topics
            }

        elif msg_type == "mux_select":
            namespace = data.get("namespace", "")
            mux_name = data.get("mux_name", "force_mux")
            topic = data.get("topic", "")
            result = await self.ros_bridge.call_mux_select(namespace, mux_name, topic)
            return {
                "type": "mux_select_response",
                "prev_topic": result["prev_topic"],
                "success": result["success"]
            }

        elif msg_type == "thruster_activate":
            namespace = data.get("namespace", "")
            candidates = data.get("candidates", ["/thrusters/activate"])

            # Scan services to find available ones
            services = self.ros_bridge.scan_services()
            service_name = self.ros_bridge.find_service_flexible(
                candidates, namespace, "std_srvs/srv/Empty"
            )

            if not service_name:
                raise Exception(f"Service not found. Tried: {candidates}")

            await self.ros_bridge.call_empty_service(service_name)
            return {
                "type": "thruster_response",
                "action": "activate",
                "success": True,
                "service": service_name
            }

        elif msg_type == "thruster_deactivate":
            namespace = data.get("namespace", "")
            candidates = data.get("candidates", ["/thrusters/deactivate"])

            service_name = self.ros_bridge.find_service_flexible(
                candidates, namespace, "std_srvs/srv/Empty"
            )

            if not service_name:
                raise Exception(f"Service not found. Tried: {candidates}")

            await self.ros_bridge.call_empty_service(service_name)
            return {
                "type": "thruster_response",
                "action": "deactivate",
                "success": True,
                "service": service_name
            }

        elif msg_type == "allocator_state":
            namespace = data.get("namespace", "")
            explicit_node_name = data.get("node_name")
            node_candidates = data.get("node_candidates")

            self.ros_bridge.scan_services()
            node_name = (
                explicit_node_name
                or self.ros_bridge.resolve_allocator_node_name(namespace)
            )
            try:
                node_name = self.ros_bridge.find_allocator_lifecycle_node(
                    namespace, node_candidates=[node_name] + (node_candidates or [])
                )
            except Exception:
                pass

            state = await self.ros_bridge.call_lifecycle_get_state(namespace, node_name)
            return {
                "type": "allocator_state_response",
                "node_name": node_name,
                "state": state,
            }

        elif msg_type == "allocator_activate":
            namespace = data.get("namespace", "")
            explicit_node_name = data.get("node_name")
            node_candidates = data.get("node_candidates")

            self.ros_bridge.scan_services()
            node_name = (
                explicit_node_name
                or self.ros_bridge.resolve_allocator_node_name(namespace)
            )
            try:
                node_name = self.ros_bridge.find_allocator_lifecycle_node(
                    namespace, node_candidates=[node_name] + (node_candidates or [])
                )
            except Exception:
                pass

            state = await self.ros_bridge.call_lifecycle_get_state(namespace, node_name)
            if state["id"] == State.PRIMARY_STATE_UNCONFIGURED:
                ok = await self.ros_bridge.call_lifecycle_change_state(
                    namespace, node_name, Transition.TRANSITION_CONFIGURE
                )
                if not ok:
                    state_now = await self.ros_bridge.call_lifecycle_get_state(namespace, node_name)
                    raise Exception(
                        f"Failed to configure allocator lifecycle node (state={state_now})"
                    )
                state = await self.ros_bridge.call_lifecycle_get_state(namespace, node_name)

            if state["id"] == State.PRIMARY_STATE_INACTIVE:
                ok = await self.ros_bridge.call_lifecycle_change_state(
                    namespace, node_name, Transition.TRANSITION_ACTIVATE
                )
                if not ok:
                    state_now = await self.ros_bridge.call_lifecycle_get_state(namespace, node_name)
                    raise Exception(
                        f"Failed to activate allocator lifecycle node (state={state_now})"
                    )

            state = await self.ros_bridge.call_lifecycle_get_state(namespace, node_name)
            return {
                "type": "allocator_response",
                "action": "activate",
                "success": True,
                "node_name": node_name,
                "state": state,
            }

        elif msg_type == "allocator_deactivate":
            namespace = data.get("namespace", "")
            explicit_node_name = data.get("node_name")
            node_candidates = data.get("node_candidates")

            self.ros_bridge.scan_services()
            node_name = (
                explicit_node_name
                or self.ros_bridge.resolve_allocator_node_name(namespace)
            )
            try:
                node_name = self.ros_bridge.find_allocator_lifecycle_node(
                    namespace, node_candidates=[node_name] + (node_candidates or [])
                )
            except Exception:
                pass

            state = await self.ros_bridge.call_lifecycle_get_state(namespace, node_name)
            if state["id"] == State.PRIMARY_STATE_ACTIVE:
                ok = await self.ros_bridge.call_lifecycle_change_state(
                    namespace, node_name, Transition.TRANSITION_DEACTIVATE
                )
                if not ok:
                    state_now = await self.ros_bridge.call_lifecycle_get_state(namespace, node_name)
                    raise Exception(
                        f"Failed to deactivate allocator lifecycle node (state={state_now})"
                    )

            state = await self.ros_bridge.call_lifecycle_get_state(namespace, node_name)
            return {
                "type": "allocator_response",
                "action": "deactivate",
                "success": True,
                "node_name": node_name,
                "state": state,
            }

        elif msg_type == "scan_services":
            services = self.ros_bridge.scan_services()
            return {
                "type": "services",
                "services": services
            }

        else:
            raise Exception(f"Unknown message type: {msg_type}")

    async def run(self):
        """Run the WebSocket server."""
        logger.info(f"Starting WebSocket server on ws://{self.host}:{self.port}")
        async with serve(self.handler, self.host, self.port):
            await asyncio.Future()  # Run forever


def run_ros_executor(bridge: ROS2Bridge, executor: MultiThreadedExecutor):
    """Run ROS 2 executor in a separate thread."""
    try:
        executor.spin()
    except Exception as e:
        logger.error(f"Executor error: {e}")


async def main():
    """Main entry point."""
    # Initialize ROS 2
    rclpy.init()

    # Create ROS 2 bridge node
    bridge = ROS2Bridge()
    executor = MultiThreadedExecutor()
    executor.add_node(bridge)

    # Run ROS executor in a separate thread
    import threading
    ros_thread = threading.Thread(target=run_ros_executor, args=(bridge, executor), daemon=True)
    ros_thread.start()

    # Create and run WebSocket server
    ws_port = bridge.get_parameter("ws_port").value
    server = WebSocketServer(bridge, port=ws_port)

    try:
        await server.run()
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
