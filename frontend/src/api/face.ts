import { request, fileRequest } from './api';

export function faceUplaodReq(method: string, params: object, name: string) {
  const url = `/face/upload/${name}`;
  return fileRequest(url, method, params);
}

export function faceDeleteReq(method: string, query_id: number) {
  const url = `/face/delete/${query_id}`;
  return request(url, method, {});
}

export function faceListReq(method: string) {
  const url = '/face/list';
  return request(url, method, {});
}
