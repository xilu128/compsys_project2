"""Offline logging regression tests; no hardware or pyserial needed for parsing."""
import csv
import io
from pathlib import Path
import signal
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
import capture_log as log
from calibrate_mag import load_samples
from summarize_log import summarize

LINE = ('P2DATA,ms=1000,acc_x=1,acc_y=2,acc_z=999,mag_x=100,mag_y=-50,mag_z=300,'
        'steps=10,heading=90.0,heading_valid=1,distance=7.0,sensor_errors=0,'
        'ble_connected=1,ble_subscribed=1,ble_errors=0')


class LoggingTests(unittest.TestCase):
    def test_fields(self):
        row = log.parse_line(LINE, '2026-09-26T10:15:32.481+08:00')
        self.assertEqual((row['device_ms'], row['acc_z_mg'], row['mag_y_mgauss']), (1000,999,-50))
        self.assertEqual((row['step_count'],row['heading_deg'],row['distance_m']), (10,90,7))
        self.assertEqual(row['sample_count'], '')
        invalid = log.parse_line(LINE.replace('heading=90.0','heading=-1.0').replace('heading_valid=1','heading_valid=0'))
        self.assertEqual((invalid['heading_deg'],invalid['heading_valid']), (-1,0))

    def test_malformed(self):
        for line in [LINE+',ms=3', LINE.replace(',steps=10',''), LINE.replace('ms=1000','ms=nan'),
                     LINE.replace('heading=90.0','heading=nan'), LINE.replace('heading=90.0','heading=360'),
                     LINE.replace('heading_valid=1','heading_valid=0'), LINE.replace('steps=10','steps=-1'),
                     LINE.replace('ble_connected=1','ble_connected=2')]:
            with self.subTest(line=line), self.assertRaises(ValueError):
                log.parse_line(line)
        self.assertIsNone(log.parse_line('P2INFO,boot'))
        self.assertIsNone(log.parse_line('P2ERROR,sensor_init_failed'))

    def test_chunks_raw_and_recovery(self):
        raw, output = io.BytesIO(), io.StringIO(newline='')
        recorder = log.Recorder(raw,output)
        data = b'P2INFO,boot\r\nP2ERROR,x\r\nP2DATA,bad\r\n'+(LINE+'\r\n').encode()+b'partial'
        for b in data:
            recorder.feed(bytes([b]))
        self.assertEqual(raw.getvalue(),data)
        self.assertEqual(recorder.samples,1)
        self.assertEqual(recorder.malformed,1)
        self.assertEqual(recorder.pending,b'partial')
        recorder.feed(b'x'*5000+b'\n'+(LINE+'\n').encode())
        self.assertEqual(recorder.samples,2)
        rows=list(csv.DictReader(io.StringIO(output.getvalue())))
        self.assertEqual(rows[0]['heading_valid'],'1')
        self.assertEqual(list(rows[0]),log.COLUMNS)

    def test_ports(self):
        bt=SimpleNamespace(device='COM3',description='Bluetooth',vid=None,pid=None)
        st=SimpleNamespace(device='COM9',description='STLink Virtual COM',vid=0x0483,pid=0x374f)
        with patch('builtins.print'):
            self.assertEqual(log.choose_port([bt,st]),'COM9')
            self.assertEqual(log.choose_port([bt],ask=lambda _: '1'),'COM3')
            self.assertEqual(log.choose_port([],requested='COM10'),'COM10')
            with self.assertRaises(ValueError): log.choose_port([])

    def test_files_and_calibration(self):
        with tempfile.TemporaryDirectory() as directory:
            directory=Path(directory)
            paths=[]
            for _ in range(2):
                rp,cp,raw,out=log.create_outputs(directory,'test')
                with raw,out:
                    log.Recorder(raw,out).feed((LINE+'\r\n').encode())
                self.assertEqual(load_samples(cp),[(100,-50,300)])
                self.assertEqual(load_samples(rp),[(100,-50,300)])
                paths.append(cp)
            self.assertNotEqual(*paths)
            with self.assertRaises(ValueError): log.create_outputs(directory,'../escape')
            legacy=directory/'legacy.log'
            legacy.write_text('steps=3 mag=100,-50,300\n',encoding='utf-8')
            self.assertEqual(load_samples(legacy),[(100,-50,300)])

    def test_summary_wrap_and_reset(self):
        a=log.parse_line(LINE,'2026-09-26T00:00:00+08:00')
        b=log.parse_line(LINE,'2026-09-26T00:00:01+08:00')
        a['device_ms']=0xffffff00
        b['device_ms']=744
        self.assertEqual(summarize([a,b])['Duration (device s)'],1)
        b['step_count']=0
        with self.assertRaises(ValueError): summarize([a,b])

    def test_ctrl_c_and_disconnect_cleanup(self):
        import serial
        for disconnect in (False,True):
            class FakePort:
                @property
                def in_waiting(self):
                    return 1000 if self.reads == 0 or disconnect else 0
                closed=False
                reads=0
                def __enter__(self): return self
                def __exit__(self,*_): self.closed=True
                def read(self,_):
                    self.reads+=1
                    if self.reads == 1:
                        if not disconnect: signal.getsignal(signal.SIGINT)(signal.SIGINT,None)
                        return (LINE+'\r\n').encode()
                    raise serial.SerialException('unplugged')
            port=FakePort()
            with tempfile.TemporaryDirectory() as directory, patch('serial.Serial',return_value=port), patch('builtins.print'):
                result=log.main(['--port','COM9','--output-dir',directory])
                self.assertEqual(result,1 if disconnect else 0)
                self.assertTrue(port.closed)
                cp=next(Path(directory).glob('*.csv'))
                with cp.open() as source: self.assertEqual(len(list(csv.DictReader(source))),1)
                self.assertEqual(next(Path(directory).glob('*.log')).read_bytes(),(LINE+'\r\n').encode())


if __name__ == '__main__':
    unittest.main()
