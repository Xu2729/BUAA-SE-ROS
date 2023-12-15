<script setup lang="ts">
import { userStore } from '@/stores/userStore';
import { computed, ref } from 'vue';
import { imgUserReq,authPwdUpdateReq,userReq } from '@/api/user';
import CryptoJS from 'crypto-js'
import {key} from '@/components/Global'

const user = userStore();

// 头像修改
let dialogAvatar = ref(false);
let dialogPwd = ref(false)
let file = ref<File | null>(null);
let pwdInfo = ref({origin_pwd:'',new_pwd:''})

async function submitAvatar() {
  await imgUserReq('post',{file:file.value})
  dialogAvatar.value = false
  let response = await userReq('get')
  user.login(response.data)
}
let imgPreview = computed(() => {
  const windowURL = window.URL || window.webkitURL;
  const ret = !(file.value instanceof File)
    ? 'https://gimg2.baidu.com/image_search/src=http%3A%2F%2Fimg2.doubanio.com%2Fview%2Fgroup_topic%2Fl%2Fpublic%2Fp515017572.jpg&refer=http%3A%2F%2Fimg2.doubanio.com&app=2002&size=f9999,10000&q=a80&n=0&g=0n&fmt=auto?sec=1684653688&t=77fc1a857d099ee9c9331a633125f65f'
    : windowURL.createObjectURL(file.value);
  return ret;
});

async function updatePwd() {
  await authPwdUpdateReq('post',{origin_pwd:CryptoJS.HmacSHA256(pwdInfo.value.origin_pwd,key).toString(),new_pwd:CryptoJS.HmacSHA256(pwdInfo.value.new_pwd,key).toString()})
  let response = await userReq('get')
  dialogPwd.value = false
  user.update(response.data)
  pwdInfo.value = {origin_pwd:'',new_pwd:''}
}
</script>

<template>
  <div class="row no-wrap q-pa-md">
    <div class="column">
      <div class="text-h6">Operation</div>
      <q-btn
        color="primary"
        label="修改头像"
        size="sm"
        dense
        class="q-mt-md"
        @click="dialogAvatar = true"
      />
      <q-dialog v-model="dialogAvatar" min-width="300px">
        <q-card class="q-pa-md">
          <div class="q-gutter-y-md column" style="min-width: 300px">
            <q-avatar size="72px" class="q-mt-xs avatar">
              <img :src="imgPreview" />
            </q-avatar>

            <q-file v-model="file" dense accept=".jpg,.jpeg,.png,.svg,image/*"/>

            <div
              style="
                display: flex;
                flex-direction: row;
                justify-content: space-between;
              "
            >
              <q-btn
                flat
                dense
                class="text-error"
                @click="dialogAvatar = false"
              >
                取消
              </q-btn>
              <q-btn flat dense class="text-primary" @click="submitAvatar" :disable="file === null">
                确认
              </q-btn>
            </div>
          </div>
        </q-card>
      </q-dialog>

      <q-dialog v-model="dialogPwd" min-width="300px">
        <q-card class="q-pa-md">
          <div class="q-gutter-y-md column" style="min-width: 300px">
            <q-input
              v-model:model-value="pwdInfo.origin_pwd"
              label="输入原始密码"
            ></q-input>
            <q-input
              v-model:model-value="pwdInfo.new_pwd"
              label="输入新密码"
            ></q-input>
            <div
              style="
                display: flex;
                flex-direction: row;
                justify-content: space-between;
              "
            >
              <q-btn
                flat
                dense
                class="text-error"
                @click="dialogPwd = false"
              >
                取消
              </q-btn>
              <q-btn flat dense class="text-primary" @click="updatePwd">
                确认
              </q-btn>
            </div>
          </div>
        </q-card>
      </q-dialog>

      <q-btn color="primary" label="修改密码" size="sm" dense class="q-mt-md" @click="dialogPwd = true"/>

      <q-btn
        color="primary"
        label="退出登录"
        size="sm"
        dense
        class="q-mt-md"
        @click="user.logout()"
      />
    </div>

    <q-separator vertical inset class="q-mx-md" />

    <div class="column items-center">
      <q-avatar size="72px" class="q-mt-xs">
        <img :src="user.avatar" />
      </q-avatar>

      <div class="text-subtitle1 q-mt-xs text-weight-bold">
        {{ user.username }}
      </div>

      <div class="q-mb-xs">{{ user.role }}</div>
    </div>
  </div>
</template>

<style scoped>
.avatar {
  left: 50%;
  transform: translateX(-50%);
  display: block;
}
</style>
