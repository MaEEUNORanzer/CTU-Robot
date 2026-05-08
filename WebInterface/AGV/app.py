#!/usr/bin/env python3
"""
修复版HTTP代理服务器
"""

from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
import socket
import json
import os
import traceback
import time

app = Flask(__name__, static_folder='.', static_url_path='')
CORS(app)

TCP_HOST = "127.0.0.1"
TCP_PORT = 8888
ENCODING = "utf-8"

def send_tcp_request(request_data):
    """向TCP服务器发送请求"""
    print(f"\n🔧 HTTP代理: 开始处理请求 {request_data.get('action')}")
    
    try:
        # 连接TCP服务器
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(10)
        sock.connect((TCP_HOST, TCP_PORT))
        
        # 发送请求
        request_str = json.dumps(request_data, ensure_ascii=False)
        print(f"🔧 发送请求长度: {len(request_str)} 字符")
        
        sock.sendall(request_str.encode(ENCODING))
        
        # 接收完整响应
        print("🔧 接收响应...")
        all_data = b""
        
        # 设置接收超时
        sock.settimeout(2)
        start_time = time.time()
        
        while True:
            try:
                chunk = sock.recv(65536)  # 64KB
                if chunk:
                    all_data += chunk
                    print(f"🔧 收到块: {len(chunk)} 字节, 总计: {len(all_data)} 字节")
                    
                    # 检查是否可能是完整JSON
                    try:
                        temp_str = all_data.decode(ENCODING, errors='ignore')
                        if temp_str.strip().endswith('}'):
                            # 尝试解析验证
                            json.loads(temp_str)
                            print("✅ 收到完整JSON")
                            break
                    except:
                        # 继续接收
                        if time.time() - start_time > 5:  # 5秒总超时
                            print("⚠️  接收超时，使用已接收数据")
                            break
                        continue
                else:
                    print("🔧 无更多数据")
                    break
            except socket.timeout:
                print("🔧 接收超时，检查是否完整")
                if len(all_data) > 0:
                    # 尝试解析已有数据
                    try:
                        temp_str = all_data.decode(ENCODING, errors='ignore')
                        if temp_str.strip():
                            json.loads(temp_str)
                            print("✅ 超时但数据完整")
                            break
                    except:
                        print("⚠️  数据不完整，但已超时")
                break
            except Exception as e:
                print(f"❌ 接收错误: {e}")
                break
        
        sock.close()
        
        if all_data:
            response_str = all_data.decode(ENCODING, errors='ignore')
            print(f"🔧 原始响应长度: {len(response_str)} 字符")
            
            # 检查响应完整性
            trimmed = response_str.strip()
            if not trimmed.endswith('}'):
                print("⚠️  响应不以}结束，可能被截断")
                # 尝试修复
                if not trimmed.endswith('}'):
                    # 查找最后一个}
                    last_brace = trimmed.rfind('}')
                    if last_brace != -1:
                        trimmed = trimmed[:last_brace+1]
                        print(f"🔧 修复: 截取到最后一个}} (位置 {last_brace})")
                    else:
                        # 没有找到}，添加一个
                        trimmed += '}'
                        print("🔧 修复: 添加结尾}")
            
            response_str = trimmed
            print(f"🔧 处理后长度: {len(response_str)} 字符")
            
            # 验证JSON
            try:
                response = json.loads(response_str)
                print(f"✅ JSON验证成功")
                return response
            except json.JSONDecodeError as e:
                print(f"❌ JSON解析失败: {e}")
                
                # 保存原始数据以便调试
                with open('proxy_error_response.txt', 'w', encoding='utf-8') as f:
                    f.write(response_str)
                print(f"📁 错误响应已保存到: proxy_error_response.txt")
                
                return {"success": False, "error": f"JSON解析失败: {str(e)}"}
        else:
            print("❌ 没有收到响应数据")
            return {"success": False, "error": "没有收到响应数据"}
            
    except Exception as e:
        print(f"❌ TCP请求失败: {str(e)}")
        return {"success": False, "error": f"TCP通信错误: {str(e)}"}

@app.route('/')
def index():
    return send_from_directory('.', 'index.html')

@app.route('/<path:path>')
def serve_static(path):
    if os.path.exists(path):
        return send_from_directory('.', path)
    return send_from_directory('.', 'index.html')

@app.route('/proxy', methods=['POST', 'OPTIONS'])
def proxy():
    if request.method == 'OPTIONS':
        return '', 200
    
    print("\n" + "="*60)
    print(f"🔧 HTTP代理: 收到请求")
    
    try:
        data = request.get_json()
        if not data:
            return jsonify({"success": False, "error": "无效的请求数据"})
        
        print(f"🔧 Action: {data.get('action')}")
        print(f"🔧 Email: {data.get('email')}")
        
        response = send_tcp_request(data)
        
        print(f"🔧 返回响应: 成功={response.get('success')}")
        if not response.get('success'):
            print(f"🔧 错误: {response.get('error')}")
        
        print("="*60)
        return jsonify(response)
        
    except Exception as e:
        print(f"❌ 处理失败: {str(e)}")
        return jsonify({"success": False, "error": f"处理失败: {str(e)}"})

if __name__ == '__main__':
    print("🚀 HTTP代理服务器启动")
    print("📡 监听地址: http://localhost:5000")
    app.run(host='0.0.0.0', port=5000, debug=False)