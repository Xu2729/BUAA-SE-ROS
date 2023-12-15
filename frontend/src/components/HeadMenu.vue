<template>
  <q-header :elevated="$q.dark.mode ? false : true" :class="headerClass" dense>
    <q-toolbar dense>
      <q-btn
        flat
        label="ROBOM"
        icon="mdi-language-r"
        @click="
          goToLink('welcome');
          tab = '';
        "
        class="btn_header"
      />
      <q-space />
      <div style="display: flex; flex-direction: row">
        <q-tabs v-model="tab" shrink stretch dense>
          <template v-for="(btn, index) in btns" :key="index">
            <q-tab
              flat
              :name="btn.label"
              :label="btn.label"
              :icon="btn.icon"
              @click="goToLink(btn.name)"
              
            />
            <q-separator vertical inset v-if="index < btns.length - 1" />
          </template>
        </q-tabs>
      </div>
      <q-space />
      <q-toggle
        v-model="logTmp"
        icon="mdi-math-log"
        color="blue-7"
        @update:model-value="setLog"
        dense
        style="padding-right: 1%"
      />
      <q-toggle
        v-model="mode.mode"
        icon="mdi-weather-sunny"
        color="yellow-7"
        :size="toggleSize"
        @update:model-value="setMode"
        dense
        style="padding-right: 1%"
      />
      <q-separator vertical inset></q-separator>
      <q-avatar>
        <img :src="user.avatar" />
        <q-menu anchor="bottom left">
          <user-list />
        </q-menu>
      </q-avatar>
    </q-toolbar>
  </q-header>
</template>

<script setup lang="ts">
import { modeStore } from '../stores/modeStore';
import { computed, onMounted, ref } from 'vue';
import { useQuasar } from 'quasar';
import { useRouter } from 'vue-router';
import UserList from './UserList.vue';
import { userReq } from '@/api/user';
import { logStore } from '@/stores/logStore';

const log = logStore()
const logTmp = ref(false)
function setLog() {
  log.setLog(logTmp.value)
}


const $q = useQuasar();
const mode = modeStore();
$q.dark.set(!mode.mode);
let toggleSize = computed(() => {
  let height: number = $q.screen.height;
  if (height > 1920) {
    return '7rem';
  } else if (height > 800) {
    return '5rem';
  } else {
    return '3rem';
  }
});
function setMode() {
  $q.dark.set(!mode.mode);
}

let headerClass = computed(() => {
  if (mode.mode) {
    return 'bg-grey-1 text-black q-gutter-y-sm';
  } else {
    return 'bg-black text-grey-1 q-gutter-y-sm';
  }
});
let tab = ref('');
let btns = [
  {
    label: '建图模式',
    icon: 'mdi-map-check-outline',
    name: 'mapping',
  },
  {
    label: '机器人控制',
    icon: 'mdi-controller-classic-outline',
    name: 'control',
  },
  {
    label: '机器人管理',
    icon: 'mdi-cog',
    name: 'settings',
  },
];

const router = useRouter();
function goToLink(name: string) {
  if (!user.isLogin) {
      tab.value = ''
  }
  router.push(name);
}

import { userStore } from '../stores/userStore';
const user = userStore();


onMounted(async () => {
  if(user.isLogin) {
    let response = await userReq('get',{})
    user.update(response.data)
  }
})
</script>

<style lang="scss" scoped>
.header {
  height: 5%;
}
.btn_header {
  height: 5%;
}

@media only screen and (min-width: 1920px) {
  .btn_header {
    font-size: 40px;
  }
}
@media only screen and (min-width: 1920px) {
  .btn_header {
    font-size: 20px;
  }
}

@media only screen and (min-width: 800px) {
  .btn_header {
    font-size: 10px;
  }
}
.q-toggle--dark .q-toggle__inner {
  color: $blue-7;
}
</style>
