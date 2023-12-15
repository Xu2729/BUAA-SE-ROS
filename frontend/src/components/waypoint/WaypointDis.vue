<template>
  <canvas
    id="map"
    :style="`height:${imgHeight}px;width:${imgWidth}px`"
  ></canvas>
</template>

<script lang="ts" setup>
import { withDefaults, defineProps, watch, onMounted, defineEmits } from 'vue';
interface autoProps {
  url: string;
  pointInfo: { x: number; y: number; theta: number; name: string }[];
  editable: boolean;
  propX: number;
  propY: number;
}

const props = withDefaults(defineProps<autoProps>(), {
  url: 'map.png',
  pointInfo: () => [],
  editable: true,
  propX: 0,
  propY: 0
});

const $emit = defineEmits(['update']);
const imgHeight = 500;
const imgWidth = 500;
let isDrawing = false;
let originHeight = 0;
let originWidth = 0;
let centerX = 0;
let centerY = 0;
let scale = 0;
let canvas: HTMLCanvasElement | null = null;
let ctx: CanvasRenderingContext2D | null = null;
let resolution = 0.03;
const img = new Image();
img.src = props.url.valueOf();
let originX: number, originY: number;
let targetX: number, targetY: number;
let tmpX: number, tmpY: number;
let rect: DOMRect | null = null;
const triangleSize = 13;
const triangleColor = '#FF0000'; // 三角形颜色
const textColor = 'blue'; // 文字颜色
const textSize = '15px Arial'; // 文字大小和字体
onMounted(() => {
  canvas = document.querySelector('#map') as HTMLCanvasElement;
  ctx = canvas.getContext('2d') as CanvasRenderingContext2D;
  canvas.width = imgWidth;
  canvas.height = imgHeight;
  const sourceAspectRatio = img.width / img.height; // 源图像的宽高比
  const targetAspectRatio = canvas.width / canvas.height; // 目标图像的宽高比
  // centerX = img.width / 2;
  centerX = props.propX !== 0 ? Math.abs(props.propX) / resolution : img.width/2;
  centerY = props.propY !== 0 ? Math.abs(props.propY) / resolution : img.height/2;
  // centerY = img.height / 2;
  originWidth = img.width;
  originHeight = img.height;
  scale =
    targetAspectRatio > sourceAspectRatio
      ? canvas.height / img.height
      : canvas.width / img.width; // 计算缩放比例

  const targetWidth = img.width * scale; // 计算目标图像的宽度
  const targetHeight = img.height * scale; // 计算目标图像的高度
  ctx.drawImage(
    img,
    0,
    0,
    img.width,
    img.height,
    0,
    0,
    targetWidth,
    targetHeight
  );
  if (props.editable) {
    canvas.addEventListener('mousedown', (event: MouseEvent) => {
      rect = (<HTMLCanvasElement>canvas).getBoundingClientRect();
      originX = event.clientX - (<DOMRect>rect).left; // 鼠标相对于 Canvas 原点的 X 坐标
      originY = event.clientY - (<DOMRect>rect).top; // 鼠标相对于 Canvas 原点的 Y 坐标
      isDrawing = true;
    });

    canvas.addEventListener('mousemove', (event: MouseEvent) => {
      if (isDrawing === true) {
        rect = (<HTMLCanvasElement>canvas).getBoundingClientRect();
        tmpX = event.clientX - (<DOMRect>rect).left; // 鼠标相对于 Canvas 原点的 X 坐标
        tmpY = event.clientY - (<DOMRect>rect).top; // 鼠标相对于 Canvas 原点的 Y 坐标

        drawLine();
      }
    });

    canvas.addEventListener('mouseup', (event: MouseEvent) => {
      isDrawing = false;
      rect = (<HTMLCanvasElement>canvas).getBoundingClientRect();
      targetX = event.clientX - (<DOMRect>rect).left; // 鼠标相对于 Canvas 原点的 X 坐标
      targetY = event.clientY - (<DOMRect>rect).top; // 鼠标相对于 Canvas 原点的 Y 坐标
      newPoint();
    });
  }
});

function newPoint() {
  const dx = targetX - originX;
  const dy = targetY - originY;
  const angle = (Math.atan2(dy, dx) + Math.PI * 2) % (Math.PI * 2);
  const newPoint = imgPos2originPos(originX, originY, angle);
  $emit('update', newPoint);
}

watch(
  () => props.pointInfo,
  (to, from) => {
    drawPoint(to);
  }
);

function drawLine() {
  drawPoint(props.pointInfo);
  (<CanvasRenderingContext2D>ctx).beginPath();
  ctx.strokeStyle = 'blue';
  ctx.lineWidth = 2;
  ctx.moveTo(originX, originY);
  ctx.lineTo(tmpX, tmpY);
  ctx.stroke();
  ctx.closePath();
}

function drawPoint(to) {
  (<CanvasRenderingContext2D>ctx).clearRect(
    0,
    0,
    (<HTMLCanvasElement>canvas).width,
    (<HTMLCanvasElement>canvas).height
  );
  (<CanvasRenderingContext2D>ctx).restore();
  const sourceAspectRatio = img.width / img.height; // 源图像的宽高比
  const targetAspectRatio = canvas.width / canvas.height; // 目标图像的宽高比
  const scale =
    targetAspectRatio > sourceAspectRatio
      ? canvas.height / img.height
      : canvas.width / img.width; // 计算缩放比例

  const targetWidth = img.width * scale; // 计算目标图像的宽度
  const targetHeight = img.height * scale; // 计算目标图像的高度
  (<CanvasRenderingContext2D>ctx).drawImage(
    img,
    0,
    0,
    img.width,
    img.height,
    0,
    0,
    targetWidth,
    targetHeight
  );
  to.forEach((ele) => {
    const Element = originPos2imgPos(ele.x, ele.y, ele.theta);
    const x1 = Element.x + triangleSize * Math.cos(Element.theta);
    const y1 = Element.y + triangleSize * Math.sin(Element.theta);
    const x2 =
      Element.x +
      triangleSize * Math.cos(Math.PI - Math.PI / 6 + Element.theta);
    const y2 =
      Element.y +
      triangleSize * Math.sin(Math.PI - Math.PI / 6 + Element.theta);
    const x3 =
      Element.x +
      triangleSize *
        Math.cos(Math.PI - Math.PI / 6 + Element.theta + Math.PI / 3);
    const y3 =
      Element.y +
      triangleSize *
        Math.sin(Math.PI - Math.PI / 6 + Element.theta + Math.PI / 3);
    // 画三角形
    (<CanvasRenderingContext2D>ctx).beginPath();
    (<CanvasRenderingContext2D>ctx).moveTo(x1, y1);
    (<CanvasRenderingContext2D>ctx).lineTo(x2, y2);
    (<CanvasRenderingContext2D>ctx).lineTo(x3, y3);
    (<CanvasRenderingContext2D>ctx).closePath();
    (<CanvasRenderingContext2D>ctx).fillStyle = triangleColor;
    (<CanvasRenderingContext2D>ctx).fill();
    // 画数字
    (<CanvasRenderingContext2D>ctx).font = textSize;
    (<CanvasRenderingContext2D>ctx).fillStyle = textColor;
    (<CanvasRenderingContext2D>ctx).textAlign = 'center';
    (<CanvasRenderingContext2D>ctx).textBaseline = 'middle';
    (<CanvasRenderingContext2D>ctx).fillText(ele.name, Element.x, Element.y);
  });
}

function originPos2imgPos(x: number, y: number, theta: number) {
  // 切换坐标系,从中心坐标系切换到左上角
  let tmpX =
    x / resolution /*将距离坐标切换成像素坐标*/ +
    centerX; /*图像左下角坐标系的坐标*/
  let tmpY =
    originHeight -
    (y / resolution /*将距离坐标切换成像素坐标*/ +
      centerY) /*图像左下角坐标系的坐标*/;
  // 角方向切换
  let newTheta =
    Math.PI * 2 -
    ((theta + Math.PI * 2) % (Math.PI * 2)) /*切换成0-2pi区间*/; /*方向调转 */
  // scale 放缩
  let newX = tmpX * scale; /**切换到新图像坐标 */
  let newY = tmpY * scale; /**切换到新图像坐标 */
  return { x: newX, y: newY, theta: newTheta };
}

function imgPos2originPos(x: number, y: number, theta: number) {
  // scale 放缩
  let tmpX = x / scale;
  let tmpY = y / scale;
  // 切换坐标系
  let newX =
    (tmpX - centerX) /*切换到图像中心坐标系*/ * resolution; /**切换到距离坐标 */
  let newY =
    (originHeight - tmpY - centerY) /*切换到图像中心坐标系*/ *
    resolution; /**切换到距离坐标 */
  // 角方向切换
  let newTheta = Math.PI * 2 - theta >= Math.PI ? -theta : Math.PI * 2 - theta;
  return { x: newX, y: newY, theta: newTheta };
}
onMounted(() => {
  setTimeout(() => {
    drawPoint(props.pointInfo);
  }, 800);
});
</script>

<style scoped></style>
