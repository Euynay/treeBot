// pages/camera/camera.js
const app = getApp();

Page({

  /**
   * 页面的初始数据
   */
  data: {
    imageUrl: '', // 用于保存接收到的图像数据
    rosbridge_server_ip: '192.168.205.135',
  },

  onLoad: function() {
    // 页面加载时建立WebSocket连接
    const socket = getApp().globalData.socket;

    // 如果socket未初始化，这里需要处理初始化逻辑
    if (!socket) {
      console.error('WebSocket未初始化！');
      return;
    }

    // 监听 WebSocket 连接打开事件
    socket.onOpen(function () {
      console.log('WebSocket连接成功');
      // 向 ROS 发送订阅图像话题的消息
      const subscribeMessage = {
        op: 'subscribe',
        topic: '/web_image', // 替换为你要订阅的图像话题
      };
      socket.send({
        data: JSON.stringify(subscribeMessage),
        fail: function() {
          console.error('发送订阅消息失败');
        }
      });
    });

    // 监听 WebSocket 错误事件
    socket.onError(function (error) {
      console.error('WebSocket错误', error);
    });

    // 监听 WebSocket 连接关闭事件
    socket.onClose(function () {
      console.log('WebSocket连接已关闭');
    });

    socket.onMessage((message) => {
      //console.log('原始消息内容:', message.data); // 打印原始消息内容
      try {
        // 第一次解析：解析整个消息
        const parsedMessage = JSON.parse(message.data);
        //console.log('解析后的消息:', parsedMessage);
    
        // 第二次解析：解析msg字段中的data字符串
        if (parsedMessage.msg && typeof parsedMessage.msg.data === 'string') {
          const imageData = JSON.parse(parsedMessage.msg.data);
          //console.log('图像数据:', imageData);
    
          // 确认图像数据字段存在
          if (imageData && imageData.image) {
            //console.log('Base64图像数据:', imageData.image);
            // 更新页面数据，显示图像
            this.setData({
              imageUrl: 'data:image/jpeg;base64,' + imageData.image
            });
          } else {
            console.error('图像数据未定义或不存在');
          }
        } else {
          console.error('消息中的msg字段不存在或格式不正确');
        }
      } catch (e) {
        console.error('解析消息时发生错误', e);
      }
    });
    
    


  },


  /**
   * 生命周期函数--监听页面加载
   */
  // onLoad(options) {

  // },

  /**
   * 生命周期函数--监听页面初次渲染完成
   */
  onReady() {

  },

  /**
   * 生命周期函数--监听页面显示
   */
  onShow() {

  },

  /**
   * 生命周期函数--监听页面隐藏
   */
  onHide() {

  },

  /**
   * 生命周期函数--监听页面卸载
   */
  onUnload() {

  },

  /**
   * 页面相关事件处理函数--监听用户下拉动作
   */
  onPullDownRefresh() {

  },

  /**
   * 页面上拉触底事件的处理函数
   */
  onReachBottom() {

  },

  /**
   * 用户点击右上角分享
   */
  onShareAppMessage() {

  },


  customButtonUpStartTap:function(){
    console.log('Start');
    const socket = getApp().globalData.socket;
    const publishStartMessage = {
      op: 'publish',
      topic: '/start_robot_topic', // 替换为你要订阅的图像话题
    };
    socket.send({
      data: JSON.stringify(publishStartMessage),
      success: function() {
        console.log('成功发送消息到 ROS');
      },
      fail: function() {
        console.error('发送发布话题失败');
      }
    });
  },
  // customButtonUploadTap:function(){
  //   console.log('保存图像');
  // },
    // 按钮点击事件处理函数
    customButtonUploadTap: function() {
      console.log('保存图像');
      let that = this;
      // 首先获取图像的Base64数据
      let base64Data = that.data.imageUrl;
      // 因为不能直接保存Base64格式的图像，所以需要先转换为临时文件
      wx.getFileSystemManager().writeFile({
        filePath: wx.env.USER_DATA_PATH + '/temp_image.jpg', // 临时文件路径
        data: base64Data.slice(22), // 去掉Base64前缀
        encoding: 'base64', // 指定编码格式为base64
        success: function(res) {
          // 使用临时文件路径保存图像到相册
          wx.saveImageToPhotosAlbum({
            filePath: wx.env.USER_DATA_PATH + '/temp_image.jpg',
            success: function(res) {
              wx.showToast({
                title: '保存成功',
                icon: 'success',
                duration: 2000
              });
            },
            fail: function(err) {
              console.error('保存失败', err);
              // 处理没有权限的情况
              if (err.errMsg === "saveImageToPhotosAlbum:fail auth deny") {
                console.log('用户拒绝授权保存到相册');
                wx.openSetting({
                  success(settingdata) {
                    console.log(settingdata)
                    if (settingdata.authSetting['scope.writePhotosAlbum']) {
                      console.log('获取权限成功，再次点击图片保存到相册')
                    } else {
                      console.log('获取权限失败')
                    }
                  }
                })
              }
            }
          });
        },
        fail: function(err) {
          console.error('写入文件失败', err);
        }
      });
    },
  // customButtonUpTap:function(){
  //   console.log('Up');
  // },
  // 前进按钮点击事件处理函数
  customButtonUpTap: function() {
  const socket = getApp().globalData.socket;
  if (socket) {
    // 构建控制小车前进的消息
    const forwardMessage = {
      op: 'publish',
      topic: '/cmd_vel', // 小车速度控制的话题
      msg: {
        linear: {
          x: 1.0, // 前进速度值
          y: 0.0,
          z: 0.0
        },
        angular: {
          x: 0.0,
          y: 0.0,
          z: 0.0
        }
      }
    };
    // 发送消息
    socket.send({
      data: JSON.stringify(forwardMessage),
      success: function() {
        console.log('前进命令发送成功');
      },
      fail: function() {
        console.error('前进命令发送失败');
      }
    });
  } else {
    console.error('WebSocket未初始化！');
  }
},
  customButtonDownTap:function(){
    console.log('后退');
    const socket = getApp().globalData.socket;
  if (socket) {
    // 构建控制小车后退的消息
    const forwardMessage = {
      op: 'publish',
      topic: '/cmd_vel', // 小车速度控制的话题
      msg: {
        linear: {
          x: -1.0, // 后退速度值
          y: 0.0,
          z: 0.0
        },
        angular: {
          x: 0.0,
          y: 0.0,
          z: 0.0
        }
      }
    };
    // 发送消息
    socket.send({
      data: JSON.stringify(forwardMessage),
      success: function() {
        console.log('后退命令发送成功');
      },
      fail: function() {
        console.error('后退命令发送失败');
      }
    });
  } else {
    console.error('WebSocket未初始化！');
  }
  },
  customButtonRightTap:function(){
    console.log('Right');
    const socket = getApp().globalData.socket;
  if (socket) {
    // 构建控制小车右转的消息
    const forwardMessage = {
      op: 'publish',
      topic: '/cmd_vel', // 小车速度控制的话题
      msg: {
        linear: {
          x: 0.0, // 右转速度值
          y: 0.0,
          z: 0.0
        },
        angular: {
          x: 0.0,
          y: 0.0,
          z: -1.0
        }
      }
    };
    // 发送消息
    socket.send({
      data: JSON.stringify(forwardMessage),
      success: function() {
        console.log('右转命令发送成功');
      },
      fail: function() {
        console.error('右转命令发送失败');
      }
    });
  } else {
    console.error('WebSocket未初始化！');
  }
  },
  customButtonLeftTap:function(){
    console.log('Left');
    const socket = getApp().globalData.socket;
  if (socket) {
    // 构建控制小车左转的消息
    const forwardMessage = {
      op: 'publish',
      topic: '/cmd_vel', // 小车速度控制的话题
      msg: {
        linear: {
          x: 0.0, // 左转速度值
          y: 0.0,
          z: 0.0
        },
        angular: {
          x: 0.0,
          y: 0.0,
          z: 1.0
        }
      }
    };
    // 发送消息
    socket.send({
      data: JSON.stringify(forwardMessage),
      success: function() {
        console.log('左转命令发送成功');
      },
      fail: function() {
        console.error('左转命令发送失败');
      }
    });
  } else {
    console.error('WebSocket未初始化！');
  }
  },
  customButtonSpeedup:function(){
    console.log('Speedup');
  },
  customButtonSlowdown:function(){
    console.log('Slowdown');
  },
  autoImage : function(e) {
    var that = this;
    var  originalWidth  = e.detail.width;
    var originalHeight = e.detail.height;
    var imageWidth = 0;
    var imageHeight = 0;
    wx.getSystemInfo({
      complete: (res) => {
        var winWidth = res.windowWidth;
        if (originalWidth > winWidth) {
          var autoWidth = winWidth;
          var autoHeight = (autoWidth * originalHeight) / originalWidth;
          imageWidth = autoWidth + 'px';
         imageHeight = autoHeight + 'px';
        } else {
          imageWidth = originalWidth + 'px';
          imageHeight = originalHeight + 'px';
        }
        that.setData({
          imageWidth: imageWidth,
          imageHeight: imageHeight
        });
      }
    })
  }
})

// function arrayBufferToBase64(buffer) {
//   var binary = '';
//   var bytes = new Uint8Array(buffer);
//   var len = bytes.byteLength;
//   for (var i = 0; i < len; i++) {
//     binary += String.fromCharCode(bytes[i]);
//   }
//   // 使用小程序的Base64编码方法
//   return wx.base64Encode(binary);
// }