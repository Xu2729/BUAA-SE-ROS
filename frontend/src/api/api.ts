import axios, { AxiosRequestConfig } from 'axios';
import { ApiUrl } from '@/components/Global';
import { Notify } from 'quasar';
import { userStore } from '@/stores/userStore';

axios.defaults.baseURL = ApiUrl;

export async function request(
  url: string,
  method: string,
  params: object,
  timeout = 1500
) {
  const options: AxiosRequestConfig = { url, method, headers: {} };
  axios.defaults.timeout = timeout;
  const token = getSessionToken();
  const tokenObj = token
    ? { Authorization: 'Bearer ' + token }
    : { Authorization: 114514 };
  options.headers = { ...tokenObj };
  if (typeof params !== 'undefined') {
    if (method === 'get' || method === 'delete') {
      options.params = params;
    } else {
      options.data = params;
    }
  }
  axios.interceptors.response.use(
    function (response) {
      return response;
    },
    function (e) {
      if (e.response) {
        const tmp = e.response;
        switch (tmp.status) {
          case 400:
            Notify.create({
              message: tmp.data.error_msg,
              type: 'negative',
              position: 'top',
            });
            break;
          case 401:
            Notify.create({
              message: `登录过期,请重新登录`,
              type: 'negative',
              position: 'top',
            });
            logout();
            break;
          case 500:
            Notify.create({
              message: '500 Internal Server Error',
              type: 'negative',
              position: 'top',
            });
            break;
          case 404:
            Notify.create({
              message: tmp.data.error_msg,
              type: 'negative',
              position: 'top',
            });
            break;
          default:
            Notify.create({
              message: tmp.data.error_msg,
              type: 'negative',
              position: 'top',
            });
            break;
        }
      }
    }
  );
  const response = await axios.request(options);
  return response;
}

export async function fileRequest(
  url: string,
  method: string,
  params: object,
  timeout = 1500
) {
  axios.defaults.timeout = timeout;
  const options: AxiosRequestConfig = { url, method, headers: {} };
  const token = getSessionToken();
  const tokenObj = token ? { Authorization: 'Bearer ' + token } : {};
  if (typeof params !== 'undefined') {
    if (method === 'get' || method === 'GET') {
      options.params = params;
      options.responseType = 'blob';
      options.headers = {};
    } else {
      options.data = params;
      options.headers = { 'Content-Type': 'multipart/form-data', ...tokenObj };
    }
  }

  axios.interceptors.response.use(
    function (response) {
      return response;
    },
    function (e) {
      const tmp = e.response;
      switch (tmp.status) {
        case 400:
          Notify.create({
            message: tmp.data.error_msg,
            type: 'negative',
          });
          break;
        case 401:
          Notify.create({
            message: `登录过期,请重新登录`,
            type: 'negative',
            position: 'top',
          });
          logout();
          break;
        case 500:
          Notify.create({
            message: '500 Internal Server Error',
            type: 'negative',
            position: 'top',
          });
          break;
        case 404:
          Notify.create({
            message: tmp.data.error_msg,
            type: 'negative',
            position: 'top',
          });
          break;
        default:
          Notify.create({
            message: tmp.data.error_msg,
            type: 'negative',
            position: 'top',
          });
          break;
      }
    }
  );
  const response = await axios.request(options);
  return response;
}

function getSessionToken() {
  const user = userStore();
  return user.token;
}

function logout() {
  const user = userStore();
  user.logout();
}
