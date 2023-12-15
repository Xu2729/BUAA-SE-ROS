<template>
  <q-layout view="hHh Lpr lfr">
    <HeadMenu></HeadMenu>
    <q-page-container >
      <q-separator v-if="$q.dark.mode ? true : false"></q-separator>
      <div class="banner">
        <img src="@/assets/left.svg" class="left-image" />

        <img src="@/assets/right.svg" class="right-image" />
        <div class="card" :flat="$q.dark.mode ? true : false">
          <router-view style="margin-top: 20%;"/>
        </div>
      </div>
    </q-page-container>
  </q-layout>
</template>

<script setup lang="ts">
import HeadMenu from '@/components/HeadMenu.vue';
import { useQuasar } from 'quasar';

const $q = useQuasar();
</script>

<style scoped lang="scss">
@import 'vfonts/Lato.css';
@import '@fontsource/zcool-xiaowei/400.css';
@import '@fontsource/noto-serif-jp/400.css';
.hljs-blob-num::before {
  content: attr(data-line-number);
}

*::-webkit-scrollbar {
  width: var(--n-scrollbar-width);
  height: var(--n-scrollbar-height);
}

*::-webkit-scrollbar-thumb {
  background: transparent;
  border-radius: var(--n-scrollbar-border-radius);
}

*::-webkit-scrollbar-thumb:hover {
  background: var(--n-scrollbar-color);
}
.banner {
  display: flex;
  position: relative;
  text-align: center;
  flex-direction: column;
  justify-content: center;
  height: calc(100vh - 56px);
  overflow:scroll;
}
.banner::after {
  content: '';
  width: 100%;
  height: 56px;
}

.left-image {
  top: 50%;
  position: absolute;
  transform: translateY(-50%);
}
.right-image {
  top: 50%;
  position: absolute;
  transform: translateY(-50%);
}
@media only screen and (max-width: 1920px) {
  .left-image {
    right: calc(50% + 400px);
    width: calc(50% - 400px);
  }
  .right-image {
    left: calc(50% + 400px);
    width: calc(50% - 400px);
  }
}
@media only screen and (min-width: 1920px) {
  .left-image {
    left: 0;
    width: 350px;
  }
  .right-image {
    right: 0;
    width: 350px;
  }
}

@media only screen and (min-width: 4000px) {
  .left-image {
    left: 0;
    width: 600px;
  }
  .right-image {
    right: 0;
    width: 600px;
  }
}

@media only screen and (max-width: 800px) {
  .left-image {
    top: 8px;
    left: -16px;
    min-width: 100px;
    width: calc(50% - 180px);
    min-width: unset;
    transform: none;
    position: relative;
  }
  .right-image {
    width: 0%;
    height: 0%;
  }
  .banner {
    transform: none;
    position: static;
    text-align: left;
    min-height: 550px;
    padding-top: 60px;
    padding-left: 16px;
    padding-right: 16px;
    height: calc(100vh - 124px);
  }
}

.card {
  position: absolute;
  top: 50%;
  left: 50%;
  transform: translateX(-50%) translateY(-50%);
}
</style>
