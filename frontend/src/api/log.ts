import { request, fileRequest } from './api';

export function logListReq(method: string) {
  const url = '/log/list';
  return request(url, method, {});
}

export function logLatestReq(method: string) {
  const url = '/log/latest';
  return request(url, method, {});
}
