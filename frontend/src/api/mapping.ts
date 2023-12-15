import { request, fileRequest } from './api';

export function mapLaunchReq(method: string, params: object) {
  const url = `/mapping/start`;
  return request(url, method, params);
}

export function mapCancelReq(method: string, params: object) {
  const url = '/mapping/end';
  return request(url, method, params);
}

export function mapSaveReq(method: string, params: object) {
  const url = '/mapping/save';
  return request(url, method, params, 15000);
}

export function mapMoveReq(method: string, params: object) {
  const url = '/mapping/move';
  return request(url, method, params);
}
