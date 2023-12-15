import { defineStore } from 'pinia';
import { ref, computed } from 'vue';
import router from '@/router/index';
export const userStore = defineStore('user', () => {
  interface UserInfo {
    username: string;
    role: string;
    token?: string;
    avatar: string;
  }

  const defaultInfo: UserInfo = {
    username: '',
    role: '',
    avatar: '',
    token: '',
  };
  const localUser = localStorage.getItem('user');
  const user = ref<UserInfo | null>(null);
  if (localUser === null) {
    user.value = defaultInfo;
  } else {
    user.value = JSON.parse(localUser);
  }
  const avatar = computed(() => user.value.avatar);
  const username = computed(() => user.value.username);
  const isLogin = computed(() => (user.value.token ? true : false));
  const role = computed(() => user.value.role);
  const token = computed(() => user.value.token);
  function login(info: UserInfo) {
    user.value.username = info.username;
    user.value.token = info.token;
    user.value.role = info.role;
    user.value.avatar = info.avatar;
    localStorage.setItem('user', JSON.stringify(user.value));
    router.push('welcome');
  }
  function logout() {
    user.value.username = '';
    user.value.token = '';
    user.value.role = defaultInfo.role;
    user.value.avatar = defaultInfo.avatar;
    router.push('login');
    localStorage.removeItem('user');
  }

  function update(info) {
    user.value.username = info.username;
    user.value.avatar = info.avatar;
    user.value.role = info.role;
  }

  return {
    avatar,
    username,
    role,
    isLogin,
    login,
    logout,
    token,
    user,
    update,
  };
});
