import unittest
from pynmea.nmea import (NMEASentence, GPAAM, GPALM, GPAPA, GPAPB, GPBEC, GPBOD,
                         GPBWC, GPBWR, GPBWW, GPGGA, GPGLL, GPGSA, GPGSV, GPHDG,
                         GPHDT, GPZDA, GPSTN, GPRMA, GPRMB, GPRMC, GPRTE, GPR00,
                         GPTRF, GPVBW, GPVTG, GPWCV, GPWNC, GPWPL, GPXTE,
                         PGRME, PGRMZ, PGRMM)

from pynmea.utils import checksum_calc

class TestNMEAParse(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_basic_parse(self):
        parse_map = (("Latitude", "lat"),
                     ("Direction", "lat_dir"),
                     ("Longitude", "lon"),
                     ("Direction", "lon_dir"))

        p = NMEASentence(parse_map)
        p._parse("$GPGLL,3751.65,S,14507.36,E*77")

        self.assertEqual("GPGLL", p.sen_type)
        self.assertEqual(p.parts,
                          ['GPGLL', '3751.65', 'S', '14507.36', 'E'])

    def test_parse(self):
        parse_map = (("Latitude", "lat"),
                     ("Direction", "lat_dir"),
                     ("Longitude", "lon"),
                     ("Direction", "lon_dir"))

        p = NMEASentence(parse_map)
        p.parse("$GPGLL,3751.65,S,14507.36,E*77")

        self.assertEqual("GPGLL", p.sen_type)
        self.assertEqual(p.parts,
                          ['GPGLL', '3751.65', 'S', '14507.36', 'E'])
        self.assertEqual(p.lat, '3751.65')
        self.assertEqual(p.lat_dir, 'S')
        self.assertEqual(p.lon, '14507.36')
        self.assertEqual(p.lon_dir, 'E')
        self.assertEqual(p.checksum, '77')

    def test_checksum_passes(self):
        parse_map = ('Checksum', 'checksum')
        nmea_str = "$GPGLL,3751.65,S,14507.36,E*77"
        p = NMEASentence(parse_map)
        p.checksum = '77'
        p.nmea_sentence = nmea_str
        result = p.check_chksum()

        self.assertTrue(result)

    def test_checksum_fails_wrong_checksum(self):
        parse_map = ('Checksum', 'checksum')
        nmea_str = "$GPGLL,3751.65,S,14507.36,E*78"
        p = NMEASentence(parse_map)
        p.checksum = '78'
        p.nmea_sentence = nmea_str
        result = p.check_chksum()

        self.assertFalse(result)

    def test_checksum_fails_wrong_str(self):
        parse_map = ('Checksum', 'checksum')
        nmea_str = "$GPGLL,3751.65,S,14507.36,W*77"
        p = NMEASentence(parse_map)
        p.checksum = '77'
        p.nmea_sentence = nmea_str
        result = p.check_chksum()

        self.assertFalse(result)


class TestGPAAM(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map(self):
        p = GPAAM()
        p.parse("$GPAAM,A,A,0.10,N,WPTNME*32")

        self.assertEqual("GPAAM", p.sen_type)
        self.assertEqual("A", p.arrival_circ_entered)
        self.assertEqual("A", p.perp_passed)
        self.assertEqual("0.10", p.circle_rad)
        self.assertEqual("N", p.circle_rad_unit)
        self.assertEqual("WPTNME", p.waypoint_id)
        self.assertEqual("32", p.checksum)


class TestGPALM(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map(self):
        p = GPALM()
        p.parse("$GPALM,32,1,01,5,00,264A,4E,0A5E,FD3F,A11257,B8E036,536C67,2532C1,069,000*7B")

        self.assertEqual("GPALM", p.sen_type)
        self.assertEqual("32", p.total_num_msgs)
        self.assertEqual("1", p.msg_num)
        self.assertEqual("01", p.sat_prn_num)
        self.assertEqual("5", p.gps_week_num)
        self.assertEqual("00", p.sv_health)
        self.assertEqual("264A", p.eccentricity)
        self.assertEqual("4E", p.alamanac_ref_time)
        self.assertEqual("0A5E", p.inc_angle)
        self.assertEqual("FD3F", p.rate_right_asc)
        self.assertEqual("A11257", p.root_semi_major_axis)
        self.assertEqual("B8E036", p.arg_perigee)
        self.assertEqual("536C67", p.lat_asc_node)
        self.assertEqual("2532C1", p.mean_anom)
        self.assertEqual("069", p.f0_clock_param)
        self.assertEqual("000", p.f1_clock_param)
        self.assertEqual("7B", p.checksum)


class TestGPAPA(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map(self):
        p = GPAPA()
        p.parse("$GPAPA,A,A,0.10,R,N,V,V,011,M,DEST*82")

        self.assertEqual("GPAPA", p.sen_type)
        self.assertEqual("A", p.status_gen)
        self.assertEqual("A", p.status_cycle_lock)
        self.assertEqual("0.10", p.cross_track_err_mag)
        self.assertEqual("R", p.dir_steer)
        self.assertEqual("N", p.cross_track_unit)
        self.assertEqual("V", p.arr_circle_entered)
        self.assertEqual("V", p.perp_passed)
        self.assertEqual("011", p.bearing_to_dest)
        self.assertEqual("M", p.bearing_type)
        self.assertEqual("DEST", p.dest_waypoint_id)
        self.assertEqual("82", p.checksum)


class TestGPAPB(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map(self):
        p = GPAPB()
        p.parse("$GPAPB,A,A,0.10,R,N,V,V,011,M,DEST,011,M,011,M*82")

        self.assertEqual("GPAPB", p.sen_type)
        self.assertEqual("A", p.status_gen)
        self.assertEqual("A", p.status_cycle_lock)
        self.assertEqual("0.10", p.cross_track_err_mag)
        self.assertEqual("R", p.dir_steer)
        self.assertEqual("N", p.cross_track_unit)
        self.assertEqual("V", p.arr_circle_entered)
        self.assertEqual("V", p.perp_passed)
        self.assertEqual("011", p.bearing_to_dest)
        self.assertEqual("M", p.bearing_type)
        self.assertEqual("DEST", p.dest_waypoint_id)
        self.assertEqual("011", p.bearing_pres_dest)
        self.assertEqual("M", p.bearing_pres_dest_type)
        self.assertEqual("011", p.heading_to_dest)
        self.assertEqual("M", p.heading_to_dest_type)
        self.assertEqual("82", p.checksum)


class TestGPBEC(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map_1(self):
        """ No FAA mode indicator
        """
        p = GPBEC()
        p.parse("$GPBEC,081837,,,,,,T,,M,,N,*13")

        self.assertEqual("GPBEC", p.sen_type)
        self.assertEqual("081837", p.timestamp)
        self.assertEqual("", p.waypoint_lat)
        self.assertEqual("", p.waypoint_lat_dir)
        self.assertEqual("", p.waypoint_lon)
        self.assertEqual("", p.waypoint_lon_dir)
        self.assertEqual("", p.bearing_true)
        self.assertEqual("T", p.bearing_true_sym)
        self.assertEqual("", p.bearing_mag)
        self.assertEqual("M", p.bearing_mag_sym)
        self.assertEqual("", p.nautical_miles)
        self.assertEqual("N", p.nautical_miles_sym)
        self.assertEqual("", p.waypoint_id)
        self.assertEqual("13", p.checksum)

    def test_parses_map_2(self):
        """ No FAA mode indicator
        """
        p = GPBEC()
        p.parse("GPBEC,220516,5130.02,N,00046.34,W,213.8,T,218.0,M,0004.6,N,EGLM*11")

        self.assertEqual("GPBEC", p.sen_type)
        self.assertEqual("220516", p.timestamp)
        self.assertEqual("5130.02", p.waypoint_lat)
        self.assertEqual("N", p.waypoint_lat_dir)
        self.assertEqual("00046.34", p.waypoint_lon)
        self.assertEqual("W", p.waypoint_lon_dir)
        self.assertEqual("213.8", p.bearing_true)
        self.assertEqual("T", p.bearing_true_sym)
        self.assertEqual("218.0", p.bearing_mag)
        self.assertEqual("M", p.bearing_mag_sym)
        self.assertEqual("0004.6", p.nautical_miles)
        self.assertEqual("N", p.nautical_miles_sym)
        self.assertEqual("EGLM", p.waypoint_id)
        self.assertEqual("11", p.checksum)

    def test_parses_map_3(self):
        """ WITH FAA mode indicator
        """
        p = GPBEC()
        p.parse("GPBEC,220516,5130.02,N,00046.34,W,213.8,T,218.0,M,0004.6,N,EGLM,X*11")

        self.assertEqual("GPBEC", p.sen_type)
        self.assertEqual("220516", p.timestamp)
        self.assertEqual("5130.02", p.waypoint_lat)
        self.assertEqual("N", p.waypoint_lat_dir)
        self.assertEqual("00046.34", p.waypoint_lon)
        self.assertEqual("W", p.waypoint_lon_dir)
        self.assertEqual("213.8", p.bearing_true)
        self.assertEqual("T", p.bearing_true_sym)
        self.assertEqual("218.0", p.bearing_mag)
        self.assertEqual("M", p.bearing_mag_sym)
        self.assertEqual("0004.6", p.nautical_miles)
        self.assertEqual("N", p.nautical_miles_sym)
        self.assertEqual("EGLM", p.waypoint_id)
        self.assertEqual("X", p.faa_mode)
        self.assertEqual("11", p.checksum)


class TestGPBOD(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map(self):
        p = GPBOD()
        p.parse("$GPBOD,045.,T,023.,M,DEST,START")

        self.assertEqual("GPBOD", p.sen_type)
        self.assertEqual(p.parts,
                          ['GPBOD', '045.', 'T', '023.', 'M', 'DEST', 'START'])
        self.assertEqual(p.bearing_t, '045.')
        self.assertEqual(p.bearing_t_type, 'T')
        self.assertEqual(p.bearing_mag, '023.')
        self.assertEqual(p.bearing_mag_type, 'M')
        self.assertEqual(p.dest, 'DEST')
        self.assertEqual(p.start, 'START')

    def test_gets_properties(self):
        p = GPBOD()
        p.parse("$GPBOD,045.,T,023.,M,DEST,START")

        self.assertEqual(p.bearing_true, '045.,T')
        self.assertEqual(p.bearing_magnetic, '023.,M')
        self.assertEqual(p.destination, 'DEST')
        self.assertEqual(p.origin, 'START')


class TestGPBWC(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map(self):
        p = GPBWC()
        p.parse("$GPBWC,220516,5130.02,N,00046.34,W,213.8,T,218.0,M,0004.6,N,EGLM*21")

        self.assertEqual("GPBWC", p.sen_type)
        self.assertEqual("220516", p.timestamp)
        self.assertEqual("5130.02", p.lat_next)
        self.assertEqual("N", p.lat_next_direction)
        self.assertEqual("00046.34", p.lon_next)
        self.assertEqual("W", p.lon_next_direction)
        self.assertEqual("213.8", p.true_track)
        self.assertEqual("T", p.true_track_sym)
        self.assertEqual("218.0", p.mag_track)
        self.assertEqual("M", p.mag_sym)
        self.assertEqual("0004.6", p.range_next)
        self.assertEqual("N", p.range_unit)
        self.assertEqual("EGLM", p.waypoint_name)
        self.assertEqual("21", p.checksum)

    def test_checksum_passes(self):
        p = GPBWC()
        p.parse("$GPBWC,220516,5130.02,N,00046.34,W,213.8,T,218.0,M,0004.6,N,EGLM*21")

        result = p.check_chksum()
        self.assertTrue(result)


class TestGPBWR(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map(self):
        p = GPBWR()
        p.parse("$GPBWR,161102,4217.4920,N,07055.7950,W,296.9,T,311.9,M,47.664,N,0001*3E")
        self.assertEqual("GPBWR", p.sen_type)
        self.assertEqual("161102", p.timestamp)
        self.assertEqual("4217.4920", p.lat_next)
        self.assertEqual("N", p.lat_next_direction)
        self.assertEqual("07055.7950", p.lon_next)
        self.assertEqual("W", p.lon_next_direction)
        self.assertEqual("296.9", p.true_track)
        self.assertEqual("T", p.true_track_sym)
        self.assertEqual("311.9", p.mag_track)
        self.assertEqual("M", p.mag_sym)
        self.assertEqual("47.664", p.range_next)
        self.assertEqual("N", p.range_unit)
        self.assertEqual("0001", p.waypoint_name)
        self.assertEqual("3E", p.checksum)


class TestGPBWW(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map(self):
        p = GPBWW()
        p.parse("$GPBWW,x.x,T,x.x,M,c--c,c--c*ff")

        self.assertEqual("GPBWW", p.sen_type)
        self.assertEqual("x.x", p.bearing_deg_true)
        self.assertEqual("T", p.bearing_deg_true_sym)
        self.assertEqual("x.x", p.bearing_deg_mag)
        self.assertEqual("M", p.bearing_deg_mag_sym)
        self.assertEqual("c--c", p.waypoint_id_dest)
        self.assertEqual("c--c", p.waypoint_id_orig)
        self.assertEqual("ff", p.checksum)


class TestGPGGA(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map(self):
        p = GPGGA()
        p.parse("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47")

        self.assertEqual("GPGGA", p.sen_type)
        self.assertEqual("123519", p.timestamp)
        self.assertEqual("4807.038", p.latitude)
        self.assertEqual("N", p.lat_direction)
        self.assertEqual("01131.000", p.longitude)
        self.assertEqual("E", p.lon_direction)
        self.assertEqual("1", p.gps_qual)
        self.assertEqual("08", p.num_sats)
        self.assertEqual("0.9", p.horizontal_dil)
        self.assertEqual("545.4", p.antenna_altitude)
        self.assertEqual("M", p.altitude_units)
        self.assertEqual("46.9", p.geo_sep)
        self.assertEqual("M", p.geo_sep_units)
        self.assertEqual("", p.age_gps_data)
        self.assertEqual("", p.ref_station_id)
        self.assertEqual("47", p.checksum)


class TestGPGLL(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map1(self):
        p = GPGLL()
        p.parse("$GPGLL,3751.65,S,14507.36,E*77")

        self.assertEqual("GPGLL", p.sen_type)
        self.assertEqual("3751.65", p.lat)
        self.assertEqual("S", p.lat_dir)
        self.assertEqual("14507.36", p.lon)
        self.assertEqual("E", p.lon_dir)
        #self.assertEqual("", p.timestamp) # No timestamp given
        self.assertEqual("77", p.checksum)

    def test_parses_map2(self):
        p = GPGLL()
        p.parse("$GPGLL,4916.45,N,12311.12,W,225444,A")

        self.assertEqual("GPGLL", p.sen_type)
        self.assertEqual("4916.45", p.lat)
        self.assertEqual("N", p.lat_dir)
        self.assertEqual("12311.12", p.lon)
        self.assertEqual("W", p.lon_dir)
        self.assertEqual("225444", p.timestamp)
        self.assertEqual("A", p.data_valid)

    #def test_checksum_passes1(self):
        #p = GPGLL()
        #p.nmea_sentence = "$GPGLL,4916.45,N,12311.12,W,225444,A"
        #p.data_validity = 'A'
        ##p._use_data_validity = True

        #result = p.check_chksum()
        #self.assertTrue(result)

    #def test_checksum_fails1(self):
        #p = GPGLL()
        #p.nmea_sentence = "$GPGLL,4916.45,N,12311.12,W,225444,B"
        #p.checksum = 'B'
        #p._use_data_validity = True

        #result = p.check_chksum()
        #self.assertFalse(result)

    def test_checksum_passes2(self):
        p = GPGLL()
        p.nmea_sentence = "$GPGLL,3751.65,S,14507.36,E*77"
        p.checksum = '77'

        result = p.check_chksum()
        self.assertTrue(result)

    def test_checksum_fails2(self):
        p = GPGLL()
        p.nmea_sentence = "$GPGLL,3751.65,S,14507.36,E*78"
        p.checksum = '78'

        result = p.check_chksum()
        self.assertFalse(result)

    def test_gets_properties(self):
        p = GPGLL()
        p.parse("$GPGLL,3751.65,S,14507.36,E*77")

        self.assertEqual(p.latitude, float('3751.65'))
        self.assertEqual(p.longitude, float('14507.36'))
        self.assertEqual(p.lat_direction, 'South')
        self.assertEqual(p.lon_direction, 'East')
        self.assertEqual(p.checksum, "77")

class TestGPGSA(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map(self):
        p = GPGSA()
        p.parse("$GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39")

        self.assertEqual("GPGSA", p.sen_type)
        self.assertEqual("A", p.mode)
        self.assertEqual("3", p.mode_fix_type)
        self.assertEqual("04", p.sv_id01)
        self.assertEqual("05", p.sv_id02)
        self.assertEqual("", p.sv_id03)
        self.assertEqual("09", p.sv_id04)
        self.assertEqual("12", p.sv_id05)
        self.assertEqual("", p.sv_id06)
        self.assertEqual("", p.sv_id07)
        self.assertEqual("24", p.sv_id08)
        self.assertEqual("", p.sv_id09)
        self.assertEqual("", p.sv_id10)
        self.assertEqual("", p.sv_id11)
        self.assertEqual("", p.sv_id12)
        self.assertEqual("2.5", p.pdop)
        self.assertEqual("1.3", p.hdop)
        self.assertEqual("2.1", p.vdop)
        self.assertEqual("39", p.checksum)

    def test_checksum_passes(self):
        p = GPGSA()
        p.checksum = '39'
        p.nmea_sentence = "$GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39"

        result = p.check_chksum()

        self.assertTrue(result)

    def test_checksum_fails(self):
        p = GPGSA()
        p.checksum = '38'
        p.nmea_sentence = "$GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*38"

        result = p.check_chksum()

        self.assertFalse(result)


class TestGPGSV(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map(self):
        p = GPGSV()
        p.parse("$GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74")

        self.assertEqual("GPGSV", p.sen_type)
        self.assertEqual('3', p.num_messages)
        self.assertEqual('1', p.msg_num)
        self.assertEqual('11', p.num_sv_in_view)
        self.assertEqual('03', p.sv_prn_num_1)
        self.assertEqual('03', p.elevation_deg_1)
        self.assertEqual('111', p.azimuth_1)
        self.assertEqual('00', p.snr_1)
        self.assertEqual('04', p.sv_prn_num_2)
        self.assertEqual('15', p.elevation_deg_2)
        self.assertEqual('270', p.azimuth_2)
        self.assertEqual('00', p.snr_2)
        self.assertEqual('06', p.sv_prn_num_3)
        self.assertEqual('01', p.elevation_deg_3)
        self.assertEqual('010', p.azimuth_3)
        self.assertEqual('00', p.snr_3)
        self.assertEqual('13', p.sv_prn_num_4)
        self.assertEqual('06', p.elevation_deg_4)
        self.assertEqual('292', p.azimuth_4)
        self.assertEqual('00', p.snr_4)
        self.assertEqual("74", p.checksum)

    def test_checksum_passes(self):
        p = GPGSV()
        p.checksum = '74'
        p.nmea_sentence = "$GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74"

        result = p.check_chksum()

        self.assertTrue(result)

    def test_checksum_fails(self):
        p = GPGSV()
        p.checksum = '73'
        p.nmea_sentence = "$GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74"

        result = p.check_chksum()

        self.assertFalse(result)


class TestGPHDG(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map(self):
        p = GPHDG()
        p.parse("$GPHDG,190.7,,E,0.0,E*7F")

        self.assertEqual("GPHDG", p.sen_type)
        self.assertEqual("190.7", p.heading)
        self.assertEqual("", p.deviation)
        self.assertEqual("E", p.dev_dir)
        self.assertEqual("0.0", p.variation)
        self.assertEqual("E", p.var_dir)
        self.assertEqual("7F", p.checksum)

    def test_checksum_passes(self):
        p = GPHDG()
        p.checksum = '7F'
        p.nmea_sentence = "$GPHDG,190.7,,E,0.0,E*7F"

        result = p.check_chksum()

        self.assertTrue(result)

    def test_checksum_fails(self):
        p = GPHDG()
        p.checksum = '7E'
        p.nmea_sentence = "$GPHDG,190.7,,E,0.0,E*7E"

        result = p.check_chksum()

        self.assertFalse(result)


class TestGPHDT(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map(self):
        p = GPHDT()
        p.parse("$GPHDT,227.66,T*02")

        self.assertEqual("GPHDT", p.sen_type)
        self.assertEqual("227.66", p.heading)
        self.assertEqual("T", p.hdg_true)
        self.assertEqual("02", p.checksum)

    def test_checksum_passes(self):
        p = GPHDT()
        p.checksum = '02'
        p.nmea_sentence = '$GPHDT,227.66,T*02'

        result = p.check_chksum()

        self.assertTrue(result)

    def test_checksum_fails(self):
        p = GPHDT()
        p.checksum = '03'
        p.nmea_sentence = '$GPHDT,227.66,T*03'

        result = p.check_chksum()

        self.assertFalse(result)


class TestGPR00(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map_1(self):
        p = GPR00()
        p.parse("$GPR00,EGLL,EGLM,EGTB,EGUB,EGTK,MBOT,EGTB,,,,,,,*58")

        self.assertEqual("GPR00", p.sen_type)
        self.assertEqual(['EGLL', 'EGLM', 'EGTB', 'EGUB', 'EGTK', 'MBOT',
                           'EGTB', '', '', '', '', '', '', ''],
                          p.waypoint_list)
        self.assertEqual("58", p.checksum)

    def test_parses_map_2(self):
        p = GPR00()
        p.parse("$GPR00,MINST,CHATN,CHAT1,CHATW,CHATM,CHATE,003,004,005,006,007,,,*05")

        self.assertEqual("GPR00", p.sen_type)
        self.assertEqual(['MINST', 'CHATN', 'CHAT1', 'CHATW', 'CHATM', 'CHATE',
                           '003', '004', '005', '006', '007', '', '', ''],
                          p.waypoint_list)
        self.assertEqual("05", p.checksum)

    def test_checksum_passes(self):
        p = GPR00()
        p.checksum = '58'
        p.nmea_sentence = "$GPR00,EGLL,EGLM,EGTB,EGUB,EGTK,MBOT,EGTB,,,,,,,*58"

        result = p.check_chksum()

        self.assertTrue(result)

    def test_checksum_fails(self):
        p = GPR00()
        p.checksum = '57'
        p.nmea_sentence = "$GPR00,EGLL,EGLM,EGTB,EGUB,EGTK,MBOT,EGTB,,,,,,,*57"

        result = p.check_chksum()

        self.assertFalse(result)


class TestGPRMA(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map(self):
        p = GPRMA()
        p.parse("$GPRMA,A,4630.129,N,147.372,W,,,12.2,5,7,N*51")

        self.assertEqual("GPRMA", p.sen_type)
        self.assertEqual("A", p.data_status)
        self.assertEqual("4630.129", p.lat)
        self.assertEqual("N", p.lat_dir)
        self.assertEqual("147.372", p.lon)
        self.assertEqual("W", p.lon_dir)
        self.assertEqual("", p.not_used_1)
        self.assertEqual("", p.not_used_2)
        self.assertEqual("12.2", p.spd_over_grnd)
        self.assertEqual("5", p.crse_over_grnd)
        self.assertEqual("7", p.variation)
        self.assertEqual("N", p.var_dir)
        self.assertEqual("51", p.checksum)

    def test_checksum_passes(self):
        p = GPRMA()
        p.checksum = '51'
        p.nmea_sentence = '$GPRMA,A,4630.129,N,147.372,W,,,12.2,5,7,N*51'
        result = p.check_chksum()
        self.assertTrue(result)

    def test_checksum_fails(self):
        p = GPRMA()
        p.checksum = '52'
        p.nmea_sentence = '$GPRMA,A,4630.129,N,147.372,W,,,12.2,5,7,N*52'
        result = p.check_chksum()
        self.assertFalse(result)


class TestGPRMB(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map_1(self):
        p = GPRMB()
        p.parse("$GPRMB,A,0.66,L,003,004,4917.24,N,12309.57,W,001.3,052.5,000.5,V*20")

        self.assertEqual("GPRMB", p.sen_type)
        self.assertEqual("A", p.validity)
        self.assertEqual("0.66", p.cross_track_error)
        self.assertEqual("L", p.cte_correction_dir)
        self.assertEqual("003", p.origin_waypoint_id)
        self.assertEqual("004", p.dest_waypoint_id)
        self.assertEqual("4917.24", p.dest_lat)
        self.assertEqual("N", p.dest_lat_dir)
        self.assertEqual("12309.57", p.dest_lon)
        self.assertEqual("W", p.dest_lon_dir)
        self.assertEqual("001.3", p.dest_range)
        self.assertEqual("052.5", p.dest_true_bearing)
        self.assertEqual("000.5", p.dest_velocity)
        self.assertEqual("V", p.arrival_alarm)
        self.assertEqual("20", p.checksum)

    def test_parses_map_2(self):
        p = GPRMB()
        p.parse("$GPRMB,A,4.08,L,EGLL,EGLM,5130.02,N,00046.34,W,004.6,213.9,122.9,A*3D")

        self.assertEqual("GPRMB", p.sen_type)
        self.assertEqual("A", p.validity)
        self.assertEqual("4.08", p.cross_track_error)
        self.assertEqual("L", p.cte_correction_dir)
        self.assertEqual("EGLL", p.origin_waypoint_id)
        self.assertEqual("EGLM", p.dest_waypoint_id)
        self.assertEqual("5130.02", p.dest_lat)
        self.assertEqual("N", p.dest_lat_dir)
        self.assertEqual("00046.34", p.dest_lon)
        self.assertEqual("W", p.dest_lon_dir)
        self.assertEqual("004.6", p.dest_range)
        self.assertEqual("213.9", p.dest_true_bearing)
        self.assertEqual("122.9", p.dest_velocity)
        self.assertEqual("A", p.arrival_alarm)
        self.assertEqual("3D", p.checksum)

    def test_parses_map_3(self):
        p = GPRMB()
        p.parse("$GPRMB,A,x.x,a,c--c,d--d,llll.ll,e,yyyyy.yy,f,g.g,h.h,i.i,j*kk")

        self.assertEqual("GPRMB", p.sen_type)
        self.assertEqual("A", p.validity)
        self.assertEqual("x.x", p.cross_track_error)
        self.assertEqual("a", p.cte_correction_dir)
        self.assertEqual("c--c", p.origin_waypoint_id)
        self.assertEqual("d--d", p.dest_waypoint_id)
        self.assertEqual("llll.ll", p.dest_lat)
        self.assertEqual("e", p.dest_lat_dir)
        self.assertEqual("yyyyy.yy", p.dest_lon)
        self.assertEqual("f", p.dest_lon_dir)
        self.assertEqual("g.g", p.dest_range)
        self.assertEqual("h.h", p.dest_true_bearing)
        self.assertEqual("i.i", p.dest_velocity)

        # This should include the bogus checksum as the checksum is not a valid
        # hex pair and should not be stripped off
        self.assertEqual("j*kk", p.arrival_alarm)

        # There should be no checksum as it was not a valid hex pair
        self.assertFalse(hasattr(p, 'checksum'))

    def test_checksum_passes(self):
        p = GPRMB()
        p.checksum = '20'
        p.nmea_sentence = '$GPRMB,A,0.66,L,003,004,4917.24,N,12309.57,W,001.3,052.5,000.5,V*20'
        result = p.check_chksum()
        self.assertTrue(result)

    def test_checksum_fails(self):
        p = GPRMB()
        p.checksum = '21'
        p.nmea_sentence = '$GPRMB,A,0.66,L,003,004,4917.24,N,12309.57,W,001.3,052.5,000.5,V*21'
        result = p.check_chksum()
        self.assertFalse(result)


class TestGPRMC(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map_1(self):
        p = GPRMC()
        p.parse("$GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62")

        self.assertEqual("GPRMC", p.sen_type)
        self.assertEqual("081836", p.timestamp)
        self.assertEqual("A", p.data_validity)
        self.assertEqual("3751.65", p.lat)
        self.assertEqual("S", p.lat_dir)
        self.assertEqual("14507.36", p.lon)
        self.assertEqual("E", p.lon_dir)
        self.assertEqual("000.0", p.spd_over_grnd)
        self.assertEqual("360.0", p.true_course)
        self.assertEqual("130998", p.datestamp)
        self.assertEqual("011.3", p.mag_variation)
        self.assertEqual("E", p.mag_var_dir)
        self.assertEqual("62", p.checksum)

    def test_parses_map_2(self):
        p = GPRMC()
        p.parse("$GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68")

        self.assertEqual("GPRMC", p.sen_type)
        self.assertEqual("225446", p.timestamp)
        self.assertEqual("A", p.data_validity)
        self.assertEqual("4916.45", p.lat)
        self.assertEqual("N", p.lat_dir)
        self.assertEqual("12311.12", p.lon)
        self.assertEqual("W", p.lon_dir)
        self.assertEqual("000.5", p.spd_over_grnd)
        self.assertEqual("054.7", p.true_course)
        self.assertEqual("191194", p.datestamp)
        self.assertEqual("020.3", p.mag_variation)
        self.assertEqual("E", p.mag_var_dir)
        self.assertEqual("68", p.checksum)

    def test_parses_map_3(self):
        p = GPRMC()
        p.parse("$GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70")

        self.assertEqual("GPRMC", p.sen_type)
        self.assertEqual("220516", p.timestamp)
        self.assertEqual("A", p.data_validity)
        self.assertEqual("5133.82", p.lat)
        self.assertEqual("N", p.lat_dir)
        self.assertEqual("00042.24", p.lon)
        self.assertEqual("W", p.lon_dir)
        self.assertEqual("173.8", p.spd_over_grnd)
        self.assertEqual("231.8", p.true_course)
        self.assertEqual("130694", p.datestamp)
        self.assertEqual("004.2", p.mag_variation)
        self.assertEqual("W", p.mag_var_dir)
        self.assertEqual("70", p.checksum)

    def test_checksum_passes(self):
        p = GPRMC()
        p.checksum = '70'
        p.nmea_sentence = '$GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70'
        result = p.check_chksum()
        self.assertTrue(result)

    def test_checksum_fails(self):
        p = GPRMC()
        p.checksum = '71'
        p.nmea_sentence = '$GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*71'
        result = p.check_chksum()
        self.assertFalse(result)


class TestGPRTE(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map1(self):
        p = GPRTE()
        p.parse("$GPRTE,2,1,c,0,PBRCPK,PBRTO,PTELGR,PPLAND,PYAMBU,PPFAIR,PWARRN,PMORTL,PLISMR*73")

        self.assertEqual("GPRTE", p.sen_type)
        self.assertEqual("2", p.num_in_seq)
        self.assertEqual("1", p.sen_num)
        self.assertEqual("c", p.start_type)
        self.assertEqual("0", p.active_route_id)
        self.assertEqual(["PBRCPK", "PBRTO", "PTELGR", "PPLAND", "PYAMBU",
                           "PPFAIR", "PWARRN", "PMORTL", "PLISMR"],
                          p.waypoint_list)
        self.assertEqual("73", p.checksum)

    def test_parses_map2(self):
        p = GPRTE()
        p.parse("$GPRTE,2,2,c,0,PCRESY,GRYRIE,GCORIO,GWERR,GWESTG,7FED*34")
        self.assertEqual("GPRTE", p.sen_type)
        self.assertEqual("2", p.num_in_seq)
        self.assertEqual("2", p.sen_num)
        self.assertEqual("c", p.start_type)
        self.assertEqual("0", p.active_route_id)
        self.assertEqual(
            ["PCRESY", "GRYRIE", "GCORIO", "GWERR", "GWESTG", "7FED"],
            p.waypoint_list)
        self.assertEqual("34", p.checksum)

    def test_checksum_passes(self):
        p = GPRTE()
        p.checksum = "73"
        p.nmea_sentence = "$GPRTE,2,1,c,0,PBRCPK,PBRTO,PTELGR,PPLAND,PYAMBU,PPFAIR,PWARRN,PMORTL,PLISMR*73"

        result = p.check_chksum()
        self.assertTrue(result)

    def test_checksum_fails(self):
        p = GPRTE()
        p.checksum = "74"
        p.nmea_sentence = "$GPRTE,2,1,c,0,PBRCPK,PBRTO,PTELGR,PPLAND,PYAMBU,PPFAIR,PWARRN,PMORTL,PLISMR*74"

        result = p.check_chksum()
        self.assertFalse(result)


class TestGPSTN(unittest.TestCase):
    def setup(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map1(self):
        p = GPSTN()
        p.parse("$GPSTN,10")

        self.assertEqual("GPSTN", p.sen_type)
        self.assertEqual("10", p.talker_id)

    def test_parses_map2(self):
        p = GPSTN()
        p.parse("$GPSTN,10*73")

        self.assertEqual("GPSTN", p.sen_type)
        self.assertEqual("10", p.talker_id)
        self.assertEqual("73", p.checksum)


class TestGPTRF(unittest.TestCase):
    def setup(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map(self):
        p = GPTRF()
        p.parse("$GPTRF,121314.15,020112,123.321,N,0987.232,W,2.3,4.5,6.7,8.9,ABC")

        self.assertEqual("GPTRF", p.sen_type)
        self.assertEqual("121314.15", p.timestamp)
        self.assertEqual("020112", p.date)
        self.assertEqual("123.321", p.lat)
        self.assertEqual("N", p.lat_dir)
        self.assertEqual("0987.232", p.lon)
        self.assertEqual("W", p.lon_dir)
        self.assertEqual("2.3", p.ele_angle)
        self.assertEqual("4.5", p.num_iterations)
        self.assertEqual("6.7", p.num_doppler_intervals)
        self.assertEqual("8.9", p.update_dist)
        self.assertEqual("ABC", p.sat_id)


class TestGPVBW(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map(self):
        p = GPVBW()
        p.parse("$GPVBW,10.1,-0.2,A,9.8,-0.5,A*62")

        self.assertEqual("GPVBW", p.sen_type)
        self.assertEqual("10.1", p.lon_water_spd)
        self.assertEqual("-0.2", p.trans_water_spd)
        self.assertEqual("A", p.data_validity_water_spd)
        self.assertEqual("9.8", p.lon_grnd_spd)
        self.assertEqual("-0.5", p.trans_grnd_spd)
        self.assertEqual("A", p.data_validity_grnd_spd)


class TestGPVTG(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map_1(self):
        p = GPVTG()
        p.parse("$GPVTG,360.0,T,348.7,M,000.0,N,000.0,K*43")

        self.assertEqual("GPVTG", p.sen_type)
        self.assertEqual("360.0", p.true_track)
        self.assertEqual("T", p.true_track_sym)
        self.assertEqual("348.7", p.mag_track)
        self.assertEqual("M", p.mag_track_sym)
        self.assertEqual("000.0", p.spd_over_grnd_kts)
        self.assertEqual("N", p.spd_over_grnd_kts_sym)
        self.assertEqual("000.0", p.spd_over_grnd_kmph)
        self.assertEqual("K", p.spd_over_grnd_kmph_sym)
        self.assertEqual('43', p.checksum)

    def test_parses_map_2(self):
        p = GPVTG()
        p.parse("GPVTG,054.7,T,034.4,M,005.5,N,010.2,K")

        self.assertEqual("GPVTG", p.sen_type)
        self.assertEqual("054.7", p.true_track)
        self.assertEqual("T", p.true_track_sym)
        self.assertEqual("034.4", p.mag_track)
        self.assertEqual("M", p.mag_track_sym)
        self.assertEqual("005.5", p.spd_over_grnd_kts)
        self.assertEqual("N", p.spd_over_grnd_kts_sym)
        self.assertEqual("010.2", p.spd_over_grnd_kmph)
        self.assertEqual("K", p.spd_over_grnd_kmph_sym)
        self.assertFalse(hasattr(p, 'checksum'))

    def test_parses_map3(self):
        p = GPVTG()
        p.parse("$GPVTG,t,T,,,s.ss,N,s.ss,K*hh")

        self.assertEqual("GPVTG", p.sen_type)
        self.assertEqual("t", p.true_track)
        self.assertEqual("T", p.true_track_sym)
        self.assertEqual("", p.mag_track)
        self.assertEqual("", p.mag_track_sym)
        self.assertEqual("s.ss", p.spd_over_grnd_kts)
        self.assertEqual("N", p.spd_over_grnd_kts_sym)
        self.assertEqual("s.ss", p.spd_over_grnd_kmph)

        # The checksum did not get stripped off as it is invalid
        # (not a hex pair).
        self.assertEqual("K*hh", p.spd_over_grnd_kmph_sym)

        # Despite a checksum being listed in the sentence, there should NOT be
        # on on the object as 'hh' is not a valid hex pair
        self.assertFalse(hasattr(p, 'checksum'))


class TestGPZDA(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map(self):
        p = GPZDA()
        p.parse("$GPZDA,025959.000,01,01,1970,,*5B")

        self.assertEqual("GPZDA", p.sen_type)
        self.assertEqual("025959.000", p.timestamp)
        self.assertEqual("01", p.day)
        self.assertEqual("01", p.month)
        self.assertEqual("1970", p.year)
        self.assertEqual("", p.local_zone)
        self.assertEqual("", p.local_zone_minutes)
        self.assertEqual("5B", p.checksum)

    def test_checksum_passes(self):
        p = GPZDA()
        p.checksum = '5B'
        p.nmea_sentence = '$GPZDA,025959.000,01,01,1970,,*5B'

        result = p.check_chksum()

        self.assertTrue(result)

    def test_checksum_fails(self):
        p = GPZDA()
        p.checksum = 'b5'
        p.nmea_sentence = '$GPZDA,025959.000,01,01,1970,,*b5'

        result = p.check_chksum()

        self.assertFalse(result)


class TestGPWCV(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map(self):
        p = GPWCV()
        p.parse("$GPWCV,2.3,N,ABCD*1C")

        self.assertEqual("GPWCV", p.sen_type)
        self.assertEqual("2.3", p.velocity)
        self.assertEqual("N", p.vel_units)
        self.assertEqual("ABCD", p.waypoint_id)

    def test_checksum_passes(self):
        p = GPWCV()
        p.checksum = '1C'
        p.nmea_sentence = '$GPWCV,2.3,N,ABCD*1C'

        result = p.check_chksum()

        self.assertTrue(result)

    def test_checksum_fails(self):
        p = GPWCV()
        p.checksum = '1B'
        p.nmea_sentence = '$GPWCV,2.3,N,ABCD*1B'

        result = p.check_chksum()

        self.assertFalse(result)


class TestGPWNC(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map(self):
        p = GPWNC()
        p.parse("$GPWNC,1.1,N,2.2,K,c--c,c--c*ff")

        self.assertEqual("GPWNC", p.sen_type)
        self.assertEqual("1.1", p.dist_nautical_miles)
        self.assertEqual("N", p.dist_naut_unit)
        self.assertEqual("2.2", p.dist_km)
        self.assertEqual("K", p.dist_km_unit)
        self.assertEqual("c--c", p.waypoint_origin_id)
        self.assertEqual("c--c", p.waypoint_dest_id)
        self.assertEqual("ff", p.checksum)


class TestGPWPL(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_parses_map_1(self):
        p = GPWPL()
        p.parse("$GPWPL,4917.16,N,12310.64,W,003*65")

        self.assertEqual("GPWPL", p.sen_type)
        self.assertEqual("4917.16", p.lat)
        self.assertEqual("N", p.lat_dir)
        self.assertEqual("12310.64", p.lon)
        self.assertEqual("W", p.lon_dir)
        self.assertEqual("003", p.waypoint_id)
        self.assertEqual("65", p.checksum)

    def test_parses_map_2(self):
        p = GPWPL()
        p.parse("$GPWPL,5128.62,N,00027.58,W,EGLL*59")

        self.assertEqual("GPWPL", p.sen_type)
        self.assertEqual("5128.62", p.lat)
        self.assertEqual("N", p.lat_dir)
        self.assertEqual("00027.58", p.lon)
        self.assertEqual("W", p.lon_dir)
        self.assertEqual("EGLL", p.waypoint_id)
        self.assertEqual("59", p.checksum)


class TestGPXTE(unittest.TestCase):
    def setUp(Self):
        pass

    def tearDown(self):
        pass

    def test_parses_map_1(self):
        p = GPXTE()
        p.parse("$GPXTE,A,A,0.67,L,N")

        self.assertEqual("GPXTE", p.sen_type)
        self.assertEqual("A", p.warning_flag)
        self.assertEqual("A", p.lock_flag)
        self.assertEqual("0.67", p.cross_track_err_dist)
        self.assertEqual("L", p.correction_dir)
        self.assertEqual("N", p.dist_units)

    def test_parses_map_2(self):
        p = GPXTE()
        p.parse("$GPXTE,A,A,4.07,L,N*6D")

        self.assertEqual("GPXTE", p.sen_type)
        self.assertEqual("A", p.warning_flag)
        self.assertEqual("A", p.lock_flag)
        self.assertEqual("4.07", p.cross_track_err_dist)
        self.assertEqual("L", p.correction_dir)
        self.assertEqual("N", p.dist_units)
        self.assertEqual("6D", p.checksum)


class TestPGRME(unittest.TestCase):
    def test_parses_map(self):
        p = PGRME()
        p.parse("$PGRME,3.1,M,4.2,M,5.2,M*2D")

        self.assertEqual("PGRME", p.sen_type)
        self.assertEqual("3.1", p.hpe)
        self.assertEqual("M", p.hpe_unit)
        self.assertEqual("4.2", p.vpe)
        self.assertEqual("M", p.vpe_unit)
        self.assertEqual("5.2", p.osepe)
        self.assertEqual("M", p.osepe_unit)


class TestPGRMM(unittest.TestCase):
    def test_parses_map(self):
        p = PGRMM()
        p.parse("PGRMM,WGS 84*06")

        self.assertEqual("PGRMM", p.sen_type)
        self.assertEqual("WGS 84", p.datum)


class TestPGRMZ(unittest.TestCase):
    def test_parses_map(self):
        p = PGRMZ()
        p.parse("PGRMZ,492,f,3*14")

        self.assertEqual("PGRMZ", p.sen_type)
        self.assertEqual("492", p.altitude)
        self.assertEqual("f", p.altitude_unit)
        self.assertEqual("3", p.pos_fix_dim)


class TestUtils(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_checksum_calc(self):
        nmea_str1 = 'GPGLL,3751.65,S,14507.36,E'
        nmea_str2 = '$GPGLL,3751.65,S,14507.36,E'
        nmea_str3 = 'GPGLL,3751.65,S,14507.36,E*77'
        nmea_str4 = '$GPGLL,3751.65,S,14507.36,E*77'
        nmea_str5 = '$GPGLL,3751.65,S,14507.36,E*'
        nmea_str6 = 'GPGLL,3751.65,S,14507.36,E*'
        nmea_str7 = '$GPHDT,227.66,T*02'

        result1 = checksum_calc(nmea_str1)
        result2 = checksum_calc(nmea_str2)
        result3 = checksum_calc(nmea_str3)
        result4 = checksum_calc(nmea_str4)
        result5 = checksum_calc(nmea_str5)
        result6 = checksum_calc(nmea_str6)
        result7 = checksum_calc(nmea_str7)

        self.assertEqual(result1, '77')
        self.assertEqual(result2, '77')
        self.assertEqual(result3, '77')
        self.assertEqual(result4, '77')
        self.assertEqual(result5, '77')
        self.assertEqual(result6, '77')
        self.assertEqual(result7, '02')
