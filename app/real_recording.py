"""真实飞控遥测的本地记录；传输和设备控制由原控制台负责。"""
import csv,json,threading,time,uuid
from datetime import datetime,timezone
from pathlib import Path


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds')


class RealRecorder:
    """只保存通过原控制台校验的真实遥测，断线时保留已接收的记录。"""
    def __init__(self,root):
        self.root=Path(root);self.lock=threading.RLock();self.file=None;self.meta=None
        self._recover_interrupted()

    def _recover_interrupted(self):
        # 崩溃时保留完整原始文件，不用尚未刷新的计数冒充没有数据。
        for path in self.root.glob('*/metadata.json'):
            try:
                meta=json.loads(path.read_text(encoding='utf-8'))
                if meta.get('status')!='recording':continue
                count=0;bad=0;first=None;last=None
                with (path.parent/'telemetry.jsonl').open(encoding='utf-8',errors='replace') as source:
                    for line in source:
                        try:
                            packet=json.loads(line)['packet'];stamp=packet['t_us'];count+=1
                            if first is None:first=stamp
                            last=stamp
                        except (ValueError,KeyError,TypeError):bad+=1
                meta.update(status='interrupted',count=count,first_device_t_us=first,last_device_t_us=last,
                            recovered_utc=utc_now(),invalid_lines=bad)
                tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding='utf-8');tmp.replace(path)
            except (OSError,ValueError,TypeError):continue

    def start(self,transport):
        with self.lock:
            if self.file:raise ValueError('已有真实数据采集正在进行。')
            if transport not in ('usb','wifi'):raise ValueError('只能记录 USB 或 Wi-Fi 真实设备。')
            record_id=time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6]
            folder=self.root/record_id;folder.mkdir(parents=True,exist_ok=False)
            self.file=(folder/'telemetry.jsonl').open('x',encoding='utf-8',newline='\n')
            self.meta={'id':record_id,'source':'real','transport':transport,'started_utc':utc_now(),
                       'status':'recording','count':0,'firmware':None,'boot':None,
                       'first_device_t_us':None,'last_device_t_us':None}
            self._save_meta();return self.status()

    def append(self,transport,packet):
        with self.lock:
            if not self.file:raise ValueError('请先开始采集。')
            if transport!=self.meta['transport']:raise ValueError('数据来源与本次采集的连接方式不一致。')
            if not isinstance(packet,dict) or packet.get('v')!=10 or packet.get('type')!='telemetry':
                raise ValueError('不是协议 10 的遥测帧。')
            seq=packet.get('seq');ts=packet.get('t_us');boot=packet.get('boot');firmware=packet.get('firmware')
            if type(seq) is not int or seq<1 or type(ts) is not int or ts<0 or not isinstance(boot,str) or not boot or not isinstance(firmware,str):
                raise ValueError('遥测缺少有效序号、设备时间或身份。')
            if self.meta['boot'] and boot!=self.meta['boot']:raise ValueError('设备已重启，请结束当前采集后重新开始。')
            if self.meta['last_device_t_us'] is not None and ts<=self.meta['last_device_t_us']:
                raise ValueError('设备时间重复或倒退，未将该帧写入记录。')
            row={'received_utc':utc_now(),'transport':transport,'packet':packet}
            line=json.dumps(row,ensure_ascii=False,separators=(',',':'),allow_nan=False)
            self.file.write(line+'\n');self.file.flush()
            self.meta['boot']=boot;self.meta['firmware']=firmware;self.meta['count']+=1
            if self.meta['first_device_t_us'] is None:self.meta['first_device_t_us']=ts
            self.meta['last_device_t_us']=ts
            return self.meta['count']

    def stop(self,reason='stopped'):
        with self.lock:
            if not self.file:raise ValueError('当前没有进行中的真实数据采集。')
            self.file.close();self.file=None;self.meta['status']=reason;self.meta['ended_utc']=utc_now()
            self._save_meta();return self.status()

    def status(self):
        with self.lock:return dict(self.meta) if self.meta else {'status':'ready','count':0}

    def _save_meta(self):
        path=self.root/self.meta['id']/'metadata.json';tmp=path.with_suffix('.tmp')
        tmp.write_text(json.dumps(self.meta,ensure_ascii=False,indent=2),encoding='utf-8');tmp.replace(path)


def export_csv(folder):
    """导出常用遥测；完整字段仍保留在原始 JSONL 中。"""
    folder=Path(folder);path=folder/'telemetry.csv'
    with (folder/'telemetry.jsonl').open(encoding='utf-8') as source, path.open('w',encoding='utf-8-sig',newline='') as out:
        writer=None
        for line in source:
            row=json.loads(line);p=row['packet'];world=p.get('world') or {};control=p.get('control') or {}
            outputs=p.get('outputs') or {};imu=p.get('control_imu') or {};pid=control.get('pid') or {}
            def at(value,index):return value[index] if isinstance(value,list) and len(value)>index else None
            record={'received_utc':row['received_utc'],'transport':row['transport'],'device_t_us':p['t_us'],
                    'sequence':p['seq'],'firmware':p.get('firmware'),'boot':p.get('boot'),
                    'state':p.get('state'),'fault':p.get('fault'),'control_source':control.get('source_id'),
                    'world_valid':world.get('valid'),'imu_valid':imu.get('valid'),
                    'servo_azimuth_command_deg':at(outputs.get('servo_deg'),0),
                    'servo_elevation_command_deg':at(outputs.get('servo_deg'),1),'esc_pwm_us':outputs.get('esc_us'),
                    'control_error_x_deg':at(control.get('error_deg'),1),'control_error_y_deg':at(control.get('error_deg'),0),
                    'estimated_roll_deg':at(world.get('euler_deg'),0),'estimated_pitch_deg':at(world.get('euler_deg'),1),
                    'estimated_yaw_deg':at(world.get('euler_deg'),2)}
            for index,axis in enumerate(('x','y','z')):
                record['body_accel_'+axis+'_g']=at(world.get('a_body_g'),index)
                record['body_gyro_'+axis+'_dps']=at(world.get('gyro_body_dps'),index)
            for axis in ('x','y'):
                for term in ('p','i','d','output'):
                    record['pid_'+axis+'_'+term+'_deg']=(pid.get(axis) or {}).get(term+'_deg')
            if writer is None:
                writer=csv.DictWriter(out,fieldnames=list(record));writer.writeheader()
            writer.writerow(record)
    return path
