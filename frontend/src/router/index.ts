import {
  RouteLocationNormalized,
  createMemoryHistory,
  createRouter,
  createWebHashHistory,
  createWebHistory,
} from 'vue-router';

import routes from './routes';
import { userStore } from '@/stores/userStore';
import { wsStore } from '@/stores/wsStore';
import { Notify } from 'quasar';
import { rosConnectReq } from '@/api/ros';
import { isDev } from '@/components/Global';
/*
 * If not building with SSR mode, you can
 * directly export the Router instantiation;
 *
 * The function below can be async too; either use
 * async/await or return a Promise which resolves
 * with the Router instance.
 */

let connectTimes = 0;
const MAX_TIMES = 10;
let connect = false;

async function authDetect(
  to: RouteLocationNormalized,
  from: RouteLocationNormalized
) {
  const user = userStore();
  if (user.isLogin && !connect) {
    const response = await rosConnectReq('get');
    if (response && !response.data.connect) {
      Notify.create({
        message: 'ROS连接失败,有另一个用户连接机器人',
        type: 'warning',
        position: 'top',
      });
    } else if (response) {
      connect = true;
      connectWS();
    }
    if (isDev) {
      connectWS();
    }
  }
  if (!user.isLogin && to.path !== '/login') {
    Notify.create({
      message: '还未登录,请先登录',
      type: 'warning',
      position: 'top',
    });
    return { path: '/login' };
  } else if (user.isLogin && to.path === '/login') {
    return { path: '/welcome' };
  } else {
    return true;
  }
}

const createHistory = process.env.SERVER
  ? createMemoryHistory
  : process.env.VUE_ROUTER_MODE === 'history'
  ? createWebHistory
  : createWebHashHistory;

const Router = createRouter({
  scrollBehavior: () => ({ left: 0, top: 0 }),
  routes,

  // Leave this as is and make changes in quasar.conf.js instead!
  // quasar.conf.js -> build -> vueRouterMode
  // quasar.conf.js -> build -> publicPath
  history: createHistory(process.env.VUE_ROUTER_BASE),
});

Router.beforeEach(authDetect);

export default Router;

function connectWS() {
  // Connecting to ROS
  const ws = wsStore();
  const ros = new window.ROSLIB.Ros({
    url: 'ws://39.105.118.198:9090',
  });
  //判断是否连接成功并输出相应的提示消息到web控制台
  ros.on('connection', function () {
    console.log('Connected to websocket server.');
    Notify.create({
      message: '连接机器人成功',
      type: 'positive',
      position: 'top',
      timeout: 500,
    });
    ws.connectWS(ros);
  });
  ros.on('error', function (error) {
    console.log('Error connecting to websocket server: ', error);
    reconnectWs();
  });
  ros.on('close', function () {
    reconnectWs();
    console.log('Connection to websocket server closed.');
  });
}

function reconnectWs() {
  connectTimes++;
  if (connectTimes < MAX_TIMES) {
    setTimeout(function () {
      connectWS();
    }, 1000);
  } else {
    Notify.create({
      message: '连接机器人失败,请检查网络连接,并重试',
      type: 'negative',
      position: 'top',
      timeout: 500,
    });
  }
}
