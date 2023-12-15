<template>
  <img :src="url" fit="scale-down" class="kinect"/>
</template>

<script lang="ts" setup>
import "roslib/build/roslib"
import { ref,onMounted } from "vue";
import { wsStore } from "@/stores/wsStore";
import ROSLIB from "roslib";

// Publishing a Topic
// Subscribing to a Topic
const url = ref('')
let listener:ROSLIB.Topic|null = null
function subscribe() {
  const ws = wsStore()
  const ros = ws.ws
  listener = new window.ROSLIB.Topic({
    ros: ros,
    name: '/kinect2/qhd/image_color_rect/compressed',
    messageType: 'sensor_msgs/CompressedImage',
    queue_size: 1, // 必须配合throttle_rate才能生效
    throttle_rate: 5, // 猜测是获取消息间隔，单位ms，大了会更新不及时，小了浪费带宽
  });
  listener.subscribe(function (message) {
    // 将jepg图片展示
    url.value = "data:image/jpeg;base64, " + message.data
  });
}

function Unsubscribe() {
  (<ROSLIB.Topic>listener).unsubscribe()
}

onMounted(() => {
  subscribe()
})

onMounted(() => {
  let tmpFunc = window.onbeforeunload
  if(tmpFunc !== null){  
      window.onbeforeunload = () => {
      Unsubscribe()
      tmpFunc()
    }
  } else {
    window.onbeforeunload = () => {
      Unsubscribe()
    }
  }
})

</script>

<style scoped>

.kinect {
  position: fixed;
  right: 0%;
  top: 9%;
  height: 250px;
  width: 350px;
  border: 0px;
}

</style>