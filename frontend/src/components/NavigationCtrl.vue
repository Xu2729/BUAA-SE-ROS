<template>
  <div flat style="display: flex; flex-direction: row">
    <div class="q-pr-md">
      <q-card-section>
        <q-icon
          :name="
            listen
              ? 'mdi-robot-excited-outline'
              : 'mdi-robot-dead-outline'
          "
          size="130px"
        />
      </q-card-section>
      <q-card-section>
        <div class="text-subtitle1">值班控制</div>
      </q-card-section>
      <q-card-seciton>
        <q-btn
          :icon="listen ? 'mdi-pause' : 'mdi-play'"
          color="secondary"
          @click="handleInput()"
          :label="listen ? '退出' : 'beigin'"
        />
      </q-card-seciton>
    </div>
    <q-separator vertical inset size="2px" />
    <div style="min-height: 450px; min-width: 500px">
      <nav-display v-if="listen" :point-info="path"></nav-display>
    </div>
  </div>
  <q-stepper-navigation>
    <q-btn @click="back"  color="primary" label="Back" class="q-ml-sm" />
  </q-stepper-navigation>
</template>

<script setup lang="ts">
import { onUnmounted, ref, defineProps, withDefaults, defineEmits, onMounted, computed } from 'vue';
import { kinectStore } from '@/stores/kinectStore';
import { useQuasar } from 'quasar';
import NavDisplay from '@/components/rosBridge/NavDisplay.vue';
import { wsStore } from '@/stores/wsStore';
import { navPatrolReq, navStopReq, navEndReq } from '@/api/nav';


interface propInfo {
  path:{x:number,
        y:number,
        yaw:number,
        name:string,
        id:number,}[],
  loop:number
}

const kinect = kinectStore();
const props = withDefaults(defineProps<propInfo>(),{
  path: () => [],
  loop: 0
})
// 值班模式
const $emit = defineEmits(['backToStep2'])

const $q = useQuasar();

const idList = computed(() => {
  let ret:number[] = []
  props.path.forEach(ele => {
    ret.push(ele.id)
  })
  return ret
})

let listen = ref(false);
const ws = wsStore();
function handleInput() {
  if (ws.ws === null) {
    $q.notify({
      message: '未连接上机器人',
      type: 'negative',
      position: 'top',
      timeout: 1000,
    });
  } else {
    listen.value = !listen.value;
    kinect.setKinect(listen.value);
    if (listen.value) {
      navPatrolReq('post',{loop:props.loop, path:idList.value})
    } else {
      navStopReq('get',{})
    }
  }
}

function back() {
  $emit('backToStep2')
}

function beforeUnload() {
  kinect.setKinect(false);
  if (listen.value) {
    navStopReq('get',{})
  }
}

onUnmounted(() => {
  beforeUnload()
});

onMounted(() => {
  let tmpFunc = window.onbeforeunload
  if(tmpFunc !== null){  
      window.onbeforeunload = () => {
      beforeUnload()
      tmpFunc()
    }
  } else {
    window.onbeforeunload = () => {
      beforeUnload()
    }
  }
})
</script>