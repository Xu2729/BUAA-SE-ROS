<script setup lang="ts">
import "roslib/build/roslib"
import { ref,onMounted } from "vue";
import { wsStore } from "@/stores/wsStore";
import ROSLIB from "roslib";
import { useQuasar } from "quasar";
const $q = useQuasar()

let listener:ROSLIB.Topic|null = null
function subscribe() {
  const ws = wsStore()
  const ros = ws.ws
  listener = new window.ROSLIB.Topic({
    ros: ros,
    name: '/error',
    messageType: 'Tus_g5/ErrorMsg',
    queue_size: 3, // 必须配合throttle_rate才能生效
    throttle_rate: 6, // 猜测是获取消息间隔，单位ms，大了会更新不及时，小了浪费带宽
  });
  listener.subscribe(function (message) {
    // 将jepg图片展示
    let msg = message.message
    $q.notify({message:msg,
      type: 'negative',
      position: 'left',})
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