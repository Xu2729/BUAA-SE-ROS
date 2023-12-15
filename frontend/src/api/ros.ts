import { request } from './api';

export function rosConnectReq(method: string, params: object = {}) {
  const url = `/ros/connect`;
  return request(url, method, params);
}

export function rosFreeReq(method: string, params: object = {}) {
  const url = `/ros/free`;
  return request(url, method, params);
}
