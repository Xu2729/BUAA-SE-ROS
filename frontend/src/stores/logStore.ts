import { defineStore } from 'pinia';
import { ref, computed } from 'vue';

export const logStore = defineStore('log', () => {
  const show = ref(false);
  function setLog(value: boolean) {
    show.value = value;
  }
  return { show, setLog };
});
