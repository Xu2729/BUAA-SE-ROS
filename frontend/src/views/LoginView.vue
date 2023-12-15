<template>
  <q-card class="q-pa-md" v-if="state === 'register'">
    <div class="q-gutter-y-md column" style="max-width: 3000px">
      <div class="text-weight-bold text-subtitle1">欢迎注册</div>

      <div>
        <q-icon name="mdi-account-box-outline" size="lg"/>
      </div>

      <q-input
        bottom-slots
        v-model="userInfo.username"
        label="用户名"
        counter
        maxlength="8"
        dense
      >
        <template v-slot:before>
          <q-icon name="mdi-account" />
        </template>
      </q-input>

      <q-input
        bottom-slots
        v-model="userInfo.password"
        label="密码"
        counter
        maxlength="20"
        dense
        :type="eyesOpen ? 'text' : 'password'"
      >
        <template v-slot:before>
          <q-icon name="mdi-lock-question" />
        </template>

        <template v-slot:append>
          <q-icon
            :name="eyesOpen ? 'mdi-eye-outline' : 'mdi-eye-off-outline'"
            @click.stop.prevent="eyesOpen = !eyesOpen"
          />
        </template>
      </q-input>
      <q-card-actions class="btns">
        <q-btn flat @click="register"> 注册 </q-btn>
        <q-btn flat @click="state = 'login'"> 取消 </q-btn>
      </q-card-actions>
    </div>
  </q-card>
  <q-card class="q-pa-md" v-if="state === 'login'">
    <div class="q-gutter-y-md column" style="max-width: 3000px">
      <div class="text-weight-bold text-subtitle1">欢迎登录</div>

      <q-input
        hide-bottom-space
        bottom-slots
        v-model="userInfo.username"
        label="用户名"
        dense
      >
        <template v-slot:before>
          <q-icon name="mdi-account" />
        </template>
      </q-input>

      <q-input
        hide-bottom-space
        bottom-slots
        v-model="userInfo.password"
        label="密码"
        dense
        :type="eyesOpen ? 'text' : 'password'"
      >
        <template v-slot:before>
          <q-icon name="mdi-lock-question" />
        </template>

        <template v-slot:append>
          <q-icon
            :name="eyesOpen ? 'mdi-eye-outline' : 'mdi-eye-off-outline'"
            @click.stop.prevent="eyesOpen = !eyesOpen"
          />
        </template>
      </q-input>
      
      <div class="btns" >
        <q-btn flat @click="login"> 登录 </q-btn>
        <q-btn flat @click="state = 'register'"> 注册 </q-btn>
      </div>
    </div>
  </q-card>
</template>

<script setup lang="ts">
import { ref, watch } from 'vue';
import { authLoginReq, authRegisterReq } from '@/api/user';
import { int2Role } from '@/components/int2role';
import { userStore } from '@/stores/userStore';
import { wsStore } from '@/stores/wsStore';
import CryptoJS from 'crypto-js'
import {key} from '@/components/Global'
const userInfo = ref({
  username: '',
  password: '',
});

let eyesOpen = ref(false);
let user = userStore();
let state = ref('login');

const defaultAvatar =
  'https://gimg2.baidu.com/image_search/src=http%3A%2F%2Fimg2.doubanio.com%2Fview%2Fgroup_topic%2Fl%2Fpublic%2Fp515017572.jpg&refer=http%3A%2F%2Fimg2.doubanio.com&app=2002&size=f9999,10000&q=a80&n=0&g=0n&fmt=auto?sec=1684653688&t=77fc1a857d099ee9c9331a633125f65f';
watch(state, () => {
  userInfo.value.username = '';
  userInfo.value.password = '';
});

async function login() {
  const response = await authLoginReq('post', {password:CryptoJS.HmacSHA256(userInfo.value.password,key).toString(),username:userInfo.value.username});
  if (response) {
    let data: { username: string; avatar: string; token: string; role: 1 | 0 } =
      response.data;
    let loginInfo = {
      role: int2Role(data.role),
      username: userInfo.value.username,
      avatar: data.avatar,
      token: data.token,
    };
    user.login(loginInfo);
  }
  userInfo.value={password:'',username:''}

}

async function register() {
  await authRegisterReq('post', {password:CryptoJS.HmacSHA256(userInfo.value.password,key).toString(),username:userInfo.value.username});
  userInfo.value={password:'',username:''}
}
</script>

<style scoped>
@import 'vfonts/Lato.css';
@import '@fontsource/zcool-xiaowei/400.css';
@import '@fontsource/noto-serif-jp/400.css';
.avatar {
  left: 50%;
  transform: translateX(-50%);
  display: block;
}

.btns {
  flex-direction: row;
  display: flex;
  justify-content: space-between;
  margin-top: 15%;
}
</style>
