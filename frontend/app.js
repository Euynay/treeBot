// app.js
App({
  onLaunch() {
    // 展示本地存储能力
    const logs = wx.getStorageSync('logs') || []
    logs.unshift(Date.now())
    wx.setStorageSync('logs', logs)

    //全局变量保存websocket连接
    this.globalData.socket = wx.connectSocket({
      url: 'ws://' + this.globalData.rosbridge_server_ip + ':9090',
      // 替换为你的 ROS 服务器 IP 地址和端口号
      fail: function() {
        console.error('WebSocket连接失败');
      }
    });
    // 登录
    wx.login({
      success: res => {
        // 发送 res.code 到后台换取 openId, sessionKey, unionId
      }
    });
  },
  globalData: {
    userInfo: null,
    //全局变量保存websocket
    rosbridge_server_ip: '192.168.205.135', // ROS 服务器 IP 地址
    socket: null
  }
})
