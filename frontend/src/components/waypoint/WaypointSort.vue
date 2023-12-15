<template>
  <div style="display: flex; flex-direction: row" class="q-gutter-x-sm">
    <div v-if="pointInfo.length !== 0">
      <q-list dense bordered>
        <q-item class="row q-col-gutter-none" dense style="padding: 0  !important;">
          <template v-for="each in columns" :key="each.label">
            <div class="col-3">
              {{ each.label }}
            </div>
          </template>
        </q-item>
        <draggable
          :list="pointInfo"
          ghost-class="ghost"
          chosen-class="chosenClass"
          animation="300"
        >
          <template #item="{ element }">
          <q-item class="item row q-col-gutter-none" dense style="padding: 0  !important;">
            <template v-for="i in 4" :key="`${i}`">
            <div class="col-3" style="text-align: center;">
              {{i === 1? element[columns[i-1].value] : Math.floor(element[columns[i-1].value]*10000)/10000}}
            </div>
          </template>
          </q-item>
          </template>
        </draggable>
      </q-list>
      <div style="min-width: 200px">
        <q-input
          v-model.number="loop"
          label="选择循环次数,-1表示无限循环"
        />
      </div>
    </div>
    <div v-else>
      没有航点哟,去标注一下吧
    </div>
    <q-separator vertical inset size="2px"></q-separator>
    <div>
      <waypoint-dis
        :url="props.mapInfo.url"
        :pointInfo="pointInfo"
        :editable="false"
        :prop-x="mapInfo.x"
        :prop-y="mapInfo.y"
      ></waypoint-dis>
    </div>
  </div>
  <q-stepper-navigation style="padding: 0%!important;">
          <q-btn @click="emit('next',{path:pointInfo,loop:loop})" color="primary" label="continue" :disable="pointInfo.length === 0"/>
          <q-btn flat @click="emit('prev')" color="primary" label="Back" class="q-ml-sm" />
  </q-stepper-navigation>
</template>

<script lang="ts" setup>
import { defineProps, defineEmits, withDefaults, ref, onMounted } from 'vue';
import { MapInfo } from '../models';
import WaypointDis from './WaypointDis.vue';
import { pointListReq } from '@/api/waypoint';
import draggable from "vuedraggable";

interface autoProps {
  mapInfo: MapInfo;
  editable: boolean
}

const emit = defineEmits(['next','prev'])

const props = withDefaults(defineProps<autoProps>(), {
  mapInfo: {},
  editable: false
});

const columns = [
  { label: '航点名字',value:'name'},
  { label: '航点 x 坐标',value:'x'},
  { label: '航点 y 坐标',value:'y'},
  { label: '航点 z 轴旋转弧度',value:'theta'},
]

const pointInfo = ref<() => {x:number,y:number,theta:number,name:string}[]>([]);
const loop = ref(0)
onMounted(async () => {
  document.body.ondrop = function(event) {
    event.preventDefault();
    event.stopPropagation();
  };
  const response = await pointListReq('get',<number>props.mapInfo.id)
  if (response) {
    pointInfo.value = response.data.points
  } else {
    pointInfo.value = [
      {
        x:-1.03001,
        y:-0.031616,
        theta:0,
        name:'1',
        id:1
      },
      {
        name:'2',
        x:-1.00467,
        y:1.97643,
        theta:Math.PI/2,
        id:2
      }
    ]
  }
})

</script>

<style scoped lang="scss">
.itxst {
  width: 600px;
  display: flex;
}
.itxst > div:nth-of-type(1) {
  flex: 1;
}
.itxst > div:nth-of-type(2) {
  width: 270px;
  padding-left: 20px;
}
.item {
  border: solid 1px #eee;
  text-align: left;
}

.item:hover {
  cursor: move;
}
.item + .item {
}
.ghost {

}
.chosenClass {
  background-color: $lime-2;
}

</style>