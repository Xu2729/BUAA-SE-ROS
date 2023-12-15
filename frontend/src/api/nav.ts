import { request, fileRequest } from './api';

export function navStartReq(method: string, query_id: number) {
  const url = `/navigation/start/${query_id}`;
  return request(url, method, {});
}

export function navEndReq(method: string, params: object) {
  const url = '/navigation/end';
  return request(url, method, params);
}

export function navPatrolReq(method: string, params: object) {
  const url = '/navigation/patrol';
  return request(url, method, params);
}

export function navStopReq(method: string, params = {}) {
  const url = '/navigation/stop';
  return request(url, method, params);
}

export function navCurReq(method: string, params = {}) {
  const url = '/navigation/mark_curr';
  return request(url, method, params);
}

export function navMapDeleteReq(method: string, query_id: number) {
  const url = `/navigation/map/delete/${query_id}`;
  return request(url, method, {});
}
