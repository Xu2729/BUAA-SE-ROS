<template>
  <q-card flat>
    <div flat style="display: flex; flex-direction: row">
      <div>
        <q-carousel
          v-model="slide"
          class="text-black"
          control-color="purple"
          navigation
          style="min-width: 100px;height: 100%;"
        >
          <q-carousel-slide name="keyboard" class="">
            <q-card flat>
              <div style="text-align: center">
                <q-img
                  src="@/assets/microphone.svg"
                  height="130px"
                  width="130px"
                ></q-img>
              </div>
              <div>
                <div style="text-align: center"><strong>语音控制</strong></div>
              </div>
              <div
                :class="$props.tipBrief ? 'tooltip-brief' : 'tooltip-detail'"
              >
                <q-btn
                  color="primary"
                  icon="mdi-exclamation"
                  dense
                  round
                  size="sm"
                >
                  <q-tooltip anchor="bottom middle" self="top middle">
                    <template v-if="$props.tipBrief">
                      <brief-tooltip style="font-size: 15px;"></brief-tooltip>
                    </template>
                    <template v-else>
                      <detail-tooltip style="font-size: 15px;"></detail-tooltip>
                    </template>
                  </q-tooltip>
                </q-btn>
              </div>
              <div style="text-align: center">
                <q-btn
                  :icon="loading ? 'mdi-pause' : 'mdi-play'"
                  color="secondary"
                  @click="handleVoice()"
                  :label="loading ? 'end' : 'begin'"
                />
              </div>
              <q-input
                v-model:model-value="voice"
                outlined
                type="textarea"
                style="margin-top: 5%;"
              ></q-input>
              <q-btn
                rounded 
                color="primary"
                @click="confirm"
                label="确认指令"
                style="margin-top: 5%;"
              >

              </q-btn>
            </q-card>
          </q-carousel-slide>

          <q-carousel-slide name="voice">
            <q-card flat>
              <q-card-section>
                <q-img
                  src="@/assets/keyboard.svg"
                  height="130px"
                  width="130px"
                ></q-img>
              </q-card-section>
              <div>
                <div style="text-align: center"><strong>键盘控制</strong></div>
              </div>
              <div class="q-gutter-y-sm">
                <template
                  v-for="eachKey in keyCtrls"
                  :key="eachKey.keys[0].label + eachKey.keys[0].value"
                >
                  <div class="row q-col-gutter-xs">
                    <q-btn
                      v-for="key in eachKey.keys"
                      :class="`col-${eachKey.size}`"
                      :label="key.label"
                      :key="key.label"
                      :color="key.color"
                      @click="handleInput(key.value)"
                      :disable="!launchState"
                    ></q-btn>
                  </div>
                </template>
              </div>
            </q-card>
          </q-carousel-slide>
        </q-carousel>
      </div>
      <q-separator vertical inset size="2px" />
      <div style="min-height: 600px; min-width: 600px">
        <div v-if="launchState" style="margin-left: 4%">
          <slot name="map_display"></slot>
        </div>
        <div
          :class="
            (launchState ? '' : 'latitude-center') + ' q-mt-md q-gutter-x-xl'
          "
        >
          <q-btn
            size="lg"
            class="longitude-center"
            label="开始工作"
            @click="launch"
            color="primary"
            v-if="!launchState"
          ></q-btn>
          <q-btn
            v-if="launchState"
            size="lg"
            label="取消进度"
            @click="cancel"
            color="primary"
          ></q-btn>
          <q-btn
            v-if="false"
            size="lg"
            label="当前位置标为航点"
            @click="markCurDialog = true"
            color="primary"
          ></q-btn>
          <q-dialog v-model="markCurDialog" min-width="300px">
        <q-card class="q-pa-md">
          <div class="q-gutter-y-md column" style="min-width: 300px">
            <q-input v-model:model-value="curName" label="输入地图名字">

            </q-input>
            <div style="
                    display: flex;
                    flex-direction: row;
                    justify-content: space-between;
                  ">
              <q-btn flat dense class="text-error" @click="markCurDialog = false;">
                取消
              </q-btn>
              <q-btn flat dense class="text-primary" @click="markCur">
                确认
              </q-btn>
            </div>
          </div>
        </q-card>
      </q-dialog>
        </div>
      </div>
      <div class="q-pl-sm">
        <q-stepper-navigation>
          <q-btn
            flat
            @click="emit('prev')"
            color="primary"
            label="Back"
            class="q-ml-sm"
          />
        </q-stepper-navigation>
      </div>
    </div>
  </q-card>
</template>

<script setup lang="ts">
import { ref, defineEmits, defineProps, withDefaults, onUnmounted, onMounted } from 'vue';
import { useQuasar } from 'quasar';
import { ctrlCommandReq, ctrlKeyboardReq } from '@/api/userCtrl';
import BriefTooltip from './Tooltip/BriefTooltip.vue';
import DetailTooltip from './Tooltip/DetailTooltip.vue';
import { kinectStore } from '@/stores/kinectStore';
import { wsStore } from '@/stores/wsStore';
import { IatRecorder } from './iat/index';
import { navCurReq } from '@/api/nav';

interface autoProps {
  hasSave: boolean;
  tipBrief: boolean;
}

const props = withDefaults(defineProps<autoProps>(), {
  hasSave: true,
  tipBrief: true,
});

const emit = defineEmits(['launch', 'cancel', 'save', 'prev']);
let slide = ref('keyboard');
const $q = useQuasar();
const kinect = kinectStore();
// 语音输入
let loading = ref(false);
const voice = ref('');

const iat = new IatRecorder();
const MAX_SECONDS = 60;
let second = 0;
let countInterval: NodeJS.Timeout | null = null;

function voiceCtrl() {
  ctrlCommandReq('post', { command: voice.value });
}

function handleVoice() {
  loading.value = !loading.value;
  if(loading.value) {
    voiceStart()
  } else {
    voiceEnd()
  }
}

function voiceStart() {
  iat.start()
}

function voiceEnd() {
  iat.stop()

}

function confirm() {
  voiceCtrl()
}

function voiceInit() {
  iat.onWillStatusChange = function (oldStatus, status) {
    if (status === 'ing') {
      countInterval = setInterval(() => {
        second++;
        if (second >= MAX_SECONDS) {
          this.stop();
          clearInterval(<NodeJS.Timeout>countInterval);
        }
      }, 1000);
    } else if (status === 'init') {
      console.log('init');
    } else {
      clearInterval(<NodeJS.Timeout>(<unknown>countInterval));
    }
  };
  iat.onTextChange = function (text:string) {
    voice.value = text
  };
}

// 键盘输入

const keyCtrls = [
  {
    size: 4,
    keys: [
      {
        label: '左转',
        value: 'q',
        color: 'green-9',
      },
      {
        label: '前进',
        value: 'w',
        color: 'primary',
      },
      {
        label: '右转',
        value: 'r',
        color: 'green-9',
      },
    ],
  },
  {
    size: 4,
    keys: [
      {
        label: '向左',
        value: 'a',
        color: 'primary',
      },
      {
        label: '后退',
        value: 's',
        color: 'primary',
      },
      {
        label: '向右',
        value: 'd',
        color: 'primary',
      },
    ],
  },
  {
    size: 12,
    keys: [
      {
        label: '停止',
        value: 'r',
        color: 'yellow-10',
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
    ctrlKeyboardReq('post', { direction: value,speed:0 });
  } else {
    ctrlKeyboardReq('post', { direction: value,speed:directionSpeed[value] });
  }
}

let launchState = ref(false);
function launch() {
  const ws = wsStore();
  if (ws.ws === null) {
    $q.notify({
      message: '未连接上机器人',
      type: 'negative',
      position: 'top',
      timeout: 1000,
    });
  } else {
    emit('launch');
    launchState.value = true;
    kinect.setKinect(true);
  }
}

function cancel() {
  emit('cancel');
  launchState.value = false;
  kinect.setKinect(false);
}

let curName = ref('')
let markCurDialog = ref(false)
async function markCur() {
  let response = await navCurReq('post',{name:curName})
  if(response) {
    curName.value = ''
    markCurDialog.value = false
  }
}

function beforeUnload() {
  kinect.setKinect(false);
  voiceEnd()
  cancel();
}

onUnmounted(() => {
  beforeUnload()
});

onMounted(() => {
  voiceInit()
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
  top: 35%;
  left: 12%;
}

.latitude-center {
  position: relative;
  top: 50%;
  transform: translateY(-50%);
}
</style>
