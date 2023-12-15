<template>
  <div style="display: flex; flex-direction: row" class="q-gutter-x-sm">
    <div>
      <q-table
      title="Points"
      :rows="pointInfo"
      :columns="columns"
      row-key="name"
      binary-state-sort
    >
      <template v-slot:body="props">
        <q-tr :props="props">
          <q-td key="name" :props="props">
            <q-btn icon="mdi-close" color="negative" round dense @click="deleteInfo(props.row)">

            </q-btn>
          </q-td>
          <q-td key="name" :props="props">
            {{ props.row.name }}
            <q-popup-edit v-model="props.row.name" buttons v-slot="scope" v-if="editable" @save="changeName">
              <q-input v-model="scope.value" dense autofocus counter />
            </q-popup-edit>
          </q-td>
          <q-td key="x" :props="props">
            {{ props.row.x }}
          </q-td>
          <q-td key="y" :props="props">
            <div class="text-pre-wrap">{{ props.row.y }}</div>
          </q-td>
          <q-td key="theta" :props="props">
            {{ props.row.theta }}
          </q-td>
        </q-tr>
      </template>

      <template v-slot:no-data="{}">
        <div class="full-width row flex-center text-accent q-gutter-sm">
          <q-icon size="2em" name="sentiment_dissatisfied" />
          <span>
            还没有航点信息,点击地图进行设置
          </span>
        </div>
      </template>

      <template v-slot:top-right>
        <q-btn
          color="primary"
          icon="mdi-exclamation"
          dense
          round
          size="sm"
        >
          <q-tooltip anchor="bottom middle" self="top middle">
            点击地图设置坐标,鼠标拖拽方向为机器人朝向
          </q-tooltip>
        </q-btn>
      </template>

    </q-table>
  </div>
    <q-separator vertical inset size="2px"></q-separator>
    <div>
      <waypoint-dis
        :url="props.mapInfo.url"
        :pointInfo="pointInfo"
        @update="insertPoint"
        :editable="editable"
        :prop-x="mapInfo.x"
        :prop-y="mapInfo.y"
        :key="disKey"
      ></waypoint-dis>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { defineProps, withDefaults, ref, onMounted } from 'vue';
import { MapInfo } from '../models';
import { pointListReq, markPointReq, deletePointReq, renamePointReq} from '@/api/waypoint';
import WaypointDis from './WaypointDis.vue';
import { useQuasar } from 'quasar';
import { key } from '../Global';
const $q = useQuasar()
const disKey = ref(1)

interface autoProps {
  mapInfo: MapInfo;
  editable: boolean
}

const props = withDefaults(defineProps<autoProps>(), {
  mapInfo: {},
  editable: true
});

const columns = [
  {
    name: 'delete',
    label: '删除航点',
    align: 'center',
  },
  {
    name: 'name',
    label: '航点名字',
    align: 'center',
    field: row => row.name,
    format: val => `${val}`,
  },
  { name: 'x', align: 'center', label: '航点 x 坐标', field: 'x',},
  { name: 'y', label: '航点 y 坐标', field: 'y', },
  { name: 'theta', label: '航点 z 轴旋转弧度', field: 'theta' },

]

const pointInfo = ref<{x:number,y:number,theta:number,name:string,id:number}[]>([]);

async function insertPoint(info:{x:number,y:number,theta:number,id:number}) {
  let name = 0
  pointInfo.value.forEach(element => {
    if (!isNaN(+element.name)) {
      if (Number(element.name) >= name) {
        name = Number(element.name) + 1
      }
    }
  })
  pointInfo.value.push({...info,name:String(name)});
  await markPointReq('post',props.mapInfo.id,{...info,name:String(name)})
  init()
}

async function deleteInfo(info:{x:number,y:number,theta:number,name:string,id:number}) {
  await deletePointReq('delete',info.id)
  init()
}

async function changeName(newName:string,oldName:string) {
  let cur = null
  let index = 0
  let valid = true
  pointInfo.value.forEach(element => {
    if(element.name === newName) {
      $q.notify({
          message:`${newName} 已经存在,修改失败`,
          type:'warning'
      })
      valid = false
      return
    }
    if (element.name === oldName) {
      cur = element
    }
    if (cur === null) {
      index += 1
    }
  })
  if(!valid) {
    return
  }
  pointInfo.value[index].name = newName
  renamePointReq('post',{id:cur.id,name:newName})
  disKey.value ^= 1
}

async function init() {
  const response = await pointListReq('get',<number>props.mapInfo.id)
  if(response) {
    pointInfo.value = response.data.points
  }
  else {
    pointInfo.value = [
      {
        x:-1.03001,
        y:-0.031616,
        theta:0,
        name:'1',
        id:1
      },
      {
        x:-1.00467,
        y:1.97643,
        theta:Math.PI/2,
        name:'2',
        id:2
      }
    ]
  }
}

onMounted(async () => {
  init()
})

</script>
