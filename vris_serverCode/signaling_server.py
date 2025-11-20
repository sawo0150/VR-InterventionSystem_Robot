# signaling_server.py
import asyncio
import json
import websockets

# 접속한 클라이언트를 역할별로 1대씩만 관리 (필요시 확장)
clients = {"sender": None, "receiver": None}

async def handler(ws):
    role = None
    try:
        async for msg in ws:
            data = json.loads(msg)

            # 1) 처음엔 { "role": "sender" | "receiver" } 로 등록
            if "role" in data:
                role = data["role"]
                clients[role] = ws
                print(f"[JOIN] {role}")
                continue

            # 2) 그 다음부터는 offer/answer/candidate를 반대편에 그대로 전달
            if role == "sender" and clients["receiver"]:
                await clients["receiver"].send(json.dumps(data))
            elif role == "receiver" and clients["sender"]:
                await clients["sender"].send(json.dumps(data))
    except websockets.ConnectionClosed:
        pass
    finally:
        if role and clients.get(role) is ws:
            clients[role] = None
            print(f"[LEAVE] {role}")

async def main():
    async with websockets.serve(handler, "0.0.0.0", 8080):
        print("Signaling server on ws://0.0.0.0:8080")
        await asyncio.Future()

asyncio.run(main())
