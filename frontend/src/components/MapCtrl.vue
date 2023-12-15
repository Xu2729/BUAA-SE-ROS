<template>
  <q-card flat>
    <div flat style="display: flex; flex-direction: row">
      <div>
        <q-card flat>
          <q-card-section>
            <q-img src="@/assets/keyboard.svg" height="130px" width="130px"></q-img>
          </q-card-section>
          <div>
            <div style="text-align: center"><strong>键盘控制</strong></div>
          </div>
          <div class="q-gutter-y-sm">
            <template v-for="eachKey in keyCtrls" :key="eachKey.keys[0].label+eachKey.keys[0].value">
              <div class="row q-col-gutter-xs">
                <q-btn v-for="key in eachKey.keys" :class="`col-${eachKey.size}`" :label="key.label" :key="key.label"
                  :color="key.color" @click="handleInput(key.value)" :disable="!launchState"></q-btn>
              </div>
            </template>
          </div>
        </q-card>
      </div>
      <q-separator vertical inset size="2px" />
      <div style="min-height: 600px; min-width: 600px; ">
        <div v-if="launchState" style="margin-left: 4%;">
          <slot name="map_display"></slot>
        </div>
        <div :class="(launchState ? '' : 'latitude-center') + ' q-mt-md q-gutter-x-xl'
          ">
          <q-btn size="lg" class="longitude-center" label="开始工作" @click="launch" color="primary"
            v-if="!launchState"></q-btn>
          <q-btn v-if="launchState" size="lg" label="取消进度" @click="cancel" color="primary"></q-btn>
          <q-btn v-if="launchState" size="lg" label="保存结果" @click="dialogSave = true" color="primary"></q-btn>
        </div>
      </div>
      <q-dialog v-model="dialogSave" min-width="300px">
        <q-card class="q-pa-md">
          <div class="q-gutter-y-md column" style="min-width: 300px">
            <q-input v-model:model-value="mapName" label="输入地图名字">

            </q-input>
            <div style="
                    display: flex;
                    flex-direction: row;
                    justify-content: space-between;
                  ">
              <q-btn flat dense class="text-error" @click="dialogSave = false; console.log(555);">
                取消
              </q-btn>
              <q-btn flat dense class="text-primary" @click="save">
                确认
              </q-btn>
            </div>
          </div>
        </q-card>
      </q-dialog>
      <div class="q-pl-sm"></div>
    </div>
  </q-card>
</template>

<script setup lang="ts">
import { ref, defineEmits, defineProps, withDefaults, onUnmounted, onMounted } from 'vue';
import { useQuasar } from 'quasar';
import { mapMoveReq } from '@/api/mapping';
import BriefTooltip from './Tooltip/BriefTooltip.vue';
import DetailTooltip from './Tooltip/DetailTooltip.vue';
import { kinectStore } from '@/stores/kinectStore';
import { wsStore } from '@/stores/wsStore';

interface autoProps {
  hasSave: boolean;
}



const props = withDefaults(defineProps<autoProps>(), {
  hasSave: true,
});

const emit = defineEmits(['launch', 'cancel', 'save']);
const $q = useQuasar();
const kinect = kinectStore()

// 键盘输入

const keyCtrls = [
  {
    size: 4,
    keys: [
      {
        label: '左转',
        value: 'q',
        color: 'green-9'
      },
      {
        label: '前进',
        value: 'w',
        color: 'primary'
      },
      {
        label: '右转',
        value: 'e',
        color: 'green-9'
      },
    ],
  },
  {
    size: 4,
    keys: [
      {
        label: '向左',
        value: 'a',
        color: 'primary'
      },
      {
        label: '后退',
        value: 's',
        color: 'primary'
      },
      {
        label: '向右',
        value: 'd',
        color: 'primary'
      },
    ],
  },
  {
    size: 12,
    keys: [
      {
        label: '停止',
        value: 'r',
        color: 'yellow-10'
      },
    ],
  },
];

let directionSpeed = {
  'r':0,
  'w':0,
  'a':0,
  's':0,
  'd':0,
  'q':0,
  'e':0
}

const defaultSpeed = {
  'r':0,
  'w':0,
  'a':0,
  's':0,
  'd':0,
  'q':0,
  'e':0
}

const stepSpeed = 0.1
const maxSpeed = 0.3

async function handleInput(value: 'q'|'w'|'e'|'a'|'s'|'d'|'r') {
  let prevSpeed = directionSpeed[value]
  directionSpeed = {...defaultSpeed}
  directionSpeed[value] = (prevSpeed + stepSpeed) > maxSpeed ? maxSpeed : prevSpeed + stepSpeed
  if (value === 'r') {
    mapMoveReq('post', { direction: value,speed:0 });
  } else {
    mapMoveReq('post', { direction: value,speed:directionSpeed[value] });
  }
}

let launchState = ref(false);
function launch() {
  const ws = wsStore()
  if (ws.ws === null) {
    $q.notify({ message: '未连接上机器人', type: 'negative', position: 'top', timeout: 1000 })
  } else {
    emit('launch');
    launchState.value = true;
    kinect.setKinect(true)
  }
}

function cancel() {
  emit('cancel');
  launchState.value = false;
  kinect.setKinect(false)
  dialogSave.value = false;
}

const dialogSave = ref(false)

const mapName = ref('')

function save() {
  emit('save', mapName.value);
  launchState.value = false;
  kinect.setKinect(false)
  mapName.value = ''
  dialogSave.value = false;
}

function beforeUnload() {
  kinect.setKinect(false)
  if (launchState.value) {
    cancel()
  }
}

onUnmounted(() => {
  beforeUnload()
})

onMounted(() => {
  let tmpFunc = window.onbeforeunload
  if(tmpFunc !== null){  
      window.onbeforeunload = () => {
      beforeUnload()
    }
  } else {
    window.onbeforeunload = () => {
      beforeUnload()
    }
  }
})
</script>

<style scoped>
.tooltip-brief {
  position: fixed;
  top: 10%;
  left: 100px;
}

.tooltip-detail {
  position: fixed;
  top: 5%;
  left: 10px;
}

.latitude-center {
  position: relative;
  top: 50%;
  transform: translateY(-50%);
}
</style>
