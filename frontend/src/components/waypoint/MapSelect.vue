<template>
  <div style="min-width: px;display: flex;flex-direction: row;" class="q-gutter-x-md">
    <div style="min-width: 200px">
      <q-select
        v-model="tmpInfo"
        :options="infos"
        option-label="name"
        @update:model-value="update"
        :label="infos.length === 0 ? '还没有地图,去建立一个吧' :'选择地图'"
        :disable="infos.length === 0"
      />
    </div>

    <q-separator vertical inset size="2px"></q-separator>
    <div>
      <q-img
        :src="tmpInfo.url"
        fit="scale-down"
        width="300px"
        height="200px"
      ></q-img>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { defineProps, defineEmits, withDefaults, ref, onMounted } from 'vue';
import { MapInfo } from '../models';
import { mapListReq } from '@/api/waypoint';
interface autoProps {
  mapInfo: MapInfo;
}

const props = withDefaults(defineProps<autoProps>(), {
  mapInfo: {},
});

const $emit = defineEmits(['update']);
const infos = ref<MapInfo[]>([]);
const tmpInfo = ref<MapInfo>(props.mapInfo);

function update(info: MapInfo) {
  $emit('update', info);
}

onMounted(async () => {
  const response = await mapListReq('get')
  if (response) {
    infos.value = response.data.maps
  } else {
    infos.value = [
      {
        id: 1,
        name: '55',
        url: 'map.png',
      },
      {
        id: 2,
        name: '33',
        url: 'map.png',
      },
    ];
  }
});
</script>
