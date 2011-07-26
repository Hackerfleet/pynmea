from pynmea.utils import checksum_calc

class NMEASentence(object):
    """ Base sentence class. This is used to pull apart a sentence.
        It will not have any real reference to what things mean. Things that
        subclass this base class should all the additional functionality.
    """

    def __init__(self, parse_map):
        self.sen_type = None
        self.parse_map = parse_map

    def _parse(self, nmea_str):
        """ Tear the sentence apart, grabbing the name on the way. Create a
            parts attribute on the class and fill in the sentence type in
            sen_type
        """
        self.nmea_sentence = nmea_str
        self.parts = nmea_str.split(',')
        if '*' in self.parts[-1]:
            d, par, ck = self.parts.pop().rpartition('*')
            self.parts.extend([d, ck])

        self.sen_type = self.parts[0]
        if self.parts[0].startswith('$'):
            self.parts[0] = self.parts[0][1:]
        self.sen_type = self.parts[0]

    def parse(self, nmea_str):
        """ Use the parse map. Parse map should be in the format:
            (('Field name', 'field_name'),
             ('Field name', 'field_name'))

             Where the first entry in the tuple is the human readable name
             and the second is the parameter name
        """

        self._parse(nmea_str)
        assert len(self.parts[1:]) <= len(self.parse_map)
        for index, item in enumerate(self.parts[1:]):
            setattr(self, self.parse_map[index][1], item)

    def check_chksum(self):
        # If there is no checksum, raise AssertionError
        assert hasattr(self, 'checksum')

        result = checksum_calc(self.nmea_sentence)
        return (result.upper() == self.checksum.upper())




# ---------------------------------------------------------------------------- #
# Here are all the currently supported sentences. All should eventually be
# supported. They are being added as properties and other useful functions are
# implimented. Unit tests are also provided.
# ---------------------------------------------------------------------------- #

class GPBOD(NMEASentence):
    def __init__(self):
        # 045.,T,023.,M,DEST,START
        parse_map = (('Bearing True', 'bearing_t'),
                     ('Bearing True Type', 'bearing_t_type'),
                     ('Bearing Magnetic', 'bearing_mag'),
                     ('Bearing Magnetic Type', 'bearing_mag_type'),
                     ('Destination', 'dest'),
                     ('Start', 'start'))

        super(GPBOD, self).__init__(parse_map)

    @property
    def bearing_true(self):
        return ','.join([self.bearing_t, self.bearing_t_type])

    @property
    def bearing_magnetic(self):
        return ','.join([self.bearing_mag, self.bearing_mag_type])

    @property
    def destination(self):
        return self.dest

    @property
    def origin(self):
        return self.start


class GPBWC(NMEASentence):
    def __init__(self):
        parse_map = (
            ('Timestamp', 'timestamp'),
            ('Latitude of next Waypoint', 'lat_next'),
            ('Latitude of next Waypoint Direction', 'lat_next_direction'),
            ('Longitude of next Waypoint', 'lon_next'),
            ('Longitude of next Waypoint Direction', 'lon_next_direction'),
            ('True track to waypoint', 'true_track'),
            ('True Track Symbol', 'true_track_sym'),
            ('Magnetic track to waypoint', 'mag_track'),
            ('Magnetic Symbol', 'mag_sym'),
            ('Range to waypoint', 'range_next'),
            ('Unit of range', 'range_unit'),
            ('Waypoint Name', 'waypoint_name'),
            ('Checksum', 'checksum'))

        super(GPBWC, self).__init__(parse_map)


class GPBWR(NMEASentence):
    def __init__(self):
        parse_map = (
            ('Timestamp', 'timestamp'),
            ('Latitude of next Waypoint', 'lat_next'),
            ('Latitude of next Waypoint Direction', 'lat_next_direction'),
            ('Longitude of next Waypoint', 'lon_next'),
            ('Longitude of next Waypoint Direction', 'lon_next_direction'),
            ('True track to waypoint', 'true_track'),
            ('True Track Symbol', 'true_track_sym'),
            ('Magnetic track to waypoint', 'mag_track'),
            ('Magnetic Symbol', 'mag_sym'),
            ('Range to waypoint', 'range_next'),
            ('Unit of range', 'range_unit'),
            ('Waypoint Name', 'waypoint_name'),
            ('Checksum', 'checksum'))

        super(GPBWR, self).__init__(parse_map)


class GPGGA(NMEASentence):
    def __init__(self):
        parse_map = (
            ('Timestamp', 'timestamp'),
            ('Latitude', 'latitude'),
            ('Latitude Direction', 'lat_direction'),
            ('Longitude', 'longitude'),
            ('Longitude Direction', 'lon_direction'),
            ('GPS Quality Indicator', 'gps_qual'),
            ('Number of Satellites in use', 'num_sats'),
            ('Horizontal Dilution of Precision', 'horizontal_dil'),
            ('Antenna Alt above sea level (mean)', 'antenna_altitude'),
            ('Units of altitude (meters)', 'altitude_units'),
            ('Geoidal Separation', 'geo_sep'),
            ('Units of Geoidal Separation (meters)', 'geo_sep_units'),
            ('Age of Differential GPS Data (secs)', 'age_gps_data'),
            ('Differential Reference Station ID', 'ref_station_id'),
            ('Checksum', 'checksum'))

        super(GPGGA, self).__init__(parse_map)


class GPGLL(NMEASentence):
    def __init__(self):
        parse_map = (
            ('Latitude', 'lat'),
            ('Latitude Direction', 'lat_dir'),
            ('Longitude', 'lon'),
            ('Longitude Direction', 'lon_dir'),
            ('Timestamp', 'timestamp'),
            ('Checksum', 'checksum'))

        super(GPGLL, self).__init__(parse_map)
        #self.check_chksum_calc = super(GPGLL, self).check_checksum

        self._use_data_validity = False
        float

    def _parse(self, nmea_str):
        """ GPGGL Allows for a couple of different formats.
            The all have lat,direction,lon,direction

            but one may have timestamp,data_validity
            while the other has only checksum

            We shall treat data_validity as a checksum and always
            add in a timestamp field

        """
        self.nmea_sentence = nmea_str
        self.parts = nmea_str.split(',')

        if '*' in self.parts[-1]:
            # There is a checksum but no timestamp + data_validity.
            # Add an empty field for the timestamp and indicate that when
            # validating the checksum, we should use validity, not a
            # calculation
            d, par, ck = self.parts.pop().rpartition('*')
            self.parts.extend([d, '', ck])
            self._use_data_validity = True

        self.sen_type = self.parts[0]
        if self.parts[0].startswith('$'):
            self.parts[0] = self.parts[0][1:]
        self.sen_type = self.parts[0]

    def check_chksum(self):
        """ Override check_checksum. If it has been detected that
            the checksum field contains "A" for valid data and something else
            for invalid, do a check based on thsi information. Otherwise, call
            to original checksum code from the superclass
        """
        # If we are looking for an "A" character
        if self._use_data_validity:
            if self.checksum == 'A':
                return True
            else:
                return False

        else:
            # Otherwise, call the superclass version
            return super(GPGLL, self).check_chksum()

    @property
    def latitude(self):
        return float(self.lat)

    @property
    def longitude(self):
        return float(self.lon)

    @property
    def lat_direction(self):
        mapping = {'N': 'North', 'S': 'South'}
        return mapping[self.lat_dir.upper()]

    @property
    def lon_direction(self):
        mapping = {"E": "East", "W": "West"}
        return mapping[self.lon_dir.upper()]


class GPGSA(NMEASentence):
    def __init__(self):
        parse_map = (
            ('Mode', 'mode'),
            ('Mode fix type', 'mode_fix_type'),
            ('SV ID01', 'sv_id01'),
            ('SV ID02', 'sv_id02'),
            ('SV ID03', 'sv_id03'),
            ('SV ID04', 'sv_id04'),
            ('SV ID05', 'sv_id05'),
            ('SV ID06', 'sv_id06'),
            ('SV ID07', 'sv_id07'),
            ('SV ID08', 'sv_id08'),
            ('SV ID09', 'sv_id09'),
            ('SV ID10', 'sv_id10'),
            ('SV ID11', 'sv_id11'),
            ('SV ID12', 'sv_id12'),
            ('PDOP (Dilution of precision)', 'pdop'),
            ('HDOP (Horizontal DOP)', 'hdop'),
            ('VDOP (Vertical DOP)', 'vdop'),
            ('Checksum', 'checksum'))

        super(GPGSA, self).__init__(parse_map)


class GPGSV(NMEASentence):
    def __init__(self):
        parse_map = (
            ('Number of messages of type in cycle', 'num_messages'),
            ('Message Number', 'msg_num'),
            ('Total number of SVs in view', 'num_sv_in_view'),
            ('SV PRN number 1', 'sv_prn_num_1'),
            ('Elevation in degrees 1', 'elevation_deg_1'), # 90 max
            ('Azimuth, deg from true north 1', 'azimuth_1'), # 000 to 159
            ('SNR 1', 'snr_1'), # 00-99 dB
            ('SV PRN number 2', 'sv_prn_num_2'),
            ('Elevation in degrees 2', 'elevation_deg_2'), # 90 max
            ('Azimuth, deg from true north 2', 'azimuth_2'), # 000 to 159
            ('SNR 2', 'snr_2'), # 00-99 dB
            ('SV PRN number 3', 'sv_prn_num_3'),
            ('Elevation in degrees 3', 'elevation_deg_3'), # 90 max
            ('Azimuth, deg from true north 3', 'azimuth_3'), # 000 to 159
            ('SNR 3', 'snr_3'), # 00-99 dB
            ('SV PRN number 4', 'sv_prn_num_4'),
            ('Elevation in degrees 4', 'elevation_deg_4'), # 90 max
            ('Azimuth, deg from true north 4', 'azimuth_4'), # 000 to 159
            ('SNR 4', 'snr_4'),  # 00-99 dB
            ('Checksum', 'checksum'))

        super(GPGSV, self).__init__(parse_map)


class GPHDG(NMEASentence):
    """ NOTE! This is a GUESS as I cannot find an actual spec
        telling me the fields. Updates are welcome!
    """
    def __init__(self):
        parse_map = (
            ("Heading", "heading"),
            ("Deviation", "deviation"),
            ("Deviation Direction", "dev_dir"),
            ("Variation", "variation"),
            ("Variation Direction", "var_dir"),
            ("Checksum", "checksum"))

        super(GPHDG, self).__init__(parse_map)


class GPHDT(NMEASentence):
    def __init__(self):
        parse_map = (
            ("Heading", "heading"),
            ("True", "hdg_true"),
            ("Checksum", "checksum"))

        super(GPHDT, self).__init__(parse_map)


class GPSTN(NMEASentence):
    """ NOTE: No real data could be found for examples of the actual spec so
            it is a guess that there may be a checksum on the end
    """
    def __init__(self):
        parse_map = (
            ("Talker ID Number", "talker_id"), # 00 - 99
            ("Checksum", "checksum"))


        super(GPSTN, self).__init__(parse_map)


class GPZDA(NMEASentence):
    def __init__(self):
        parse_map = (
        ("Timestamp", "timestamp"), # hhmmss.ss = UTC
        ("Day", "day"), # 01 to 31
        ("Month", "month"), # 01 to 12
        ("Year", "year"), # Year = YYYY
        ("Local Zone Description", "local_zone"), # 00 to +/- 13 hours
        ("Local Zone Minutes Description", "local_zone_minutes"), # same sign as hours
        ("Checksum", "checksum"))

        super(GPZDA, self).__init__(parse_map)


#class GPAAM(NMEASentence):
    #def __init__(self):
        #super(GPAAM).__init__()


#class GPALM(NMEASentence):
    #def __init__(self):
        #super(GPALM).__init__()


#class GPAPA(NMEASentence):
    #def __init__(self):
        #super(GPAPA).__init__()


#class GPAPB(NMEASentence):
    #def __init__(self):
        #super(GPAPB).__init__()


#class GPASD(NMEASentence):
    #def __init__(self):
        #super(GPASD).__init__()


    #* $GPBEC - Bearing & Distance to Waypoint, Dead Reckoning



    #* $GPBWW - Bearing, Waypoint to Waypoint
    #* $GPDBT - Depth Below Transducer
    #* $GPDCN - Decca Position
    #* $GPDPT - Depth
    #* $GPFSI - Frequency Set Information

    #* $GPGLC - Geographic Position, Loran-C


    #* $GPGXA - TRANSIT Position


    #* $GPHSC - Heading Steering Command
    #* $GPLCD - Loran-C Signal Data
    #* $GPMTA - Air Temperature (to be phased out)
    #* $GPMTW - Water Temperature
    #* $GPMWD - Wind Direction
    #* $GPMWV - Wind Speed and Angle
    #* $GPOLN - Omega Lane Numbers
    #* $GPOSD - Own Ship Data
    #* $GPR00 - Waypoint active route (not standard)
    #* $GPRMA - Recommended Minimum Specific Loran-C Data
    #* $GPRMB - Recommended Minimum Navigation Information
    #* $GPRMC - Recommended Minimum Specific GPS/TRANSIT Data
    #* $GPROT - Rate of Turn
    #* $GPRPM - Revolutions
    #* $GPRSA - Rudder Sensor Angle
    #* $GPRSD - RADAR System Data
    #* $GPRTE - Routes
    #* $GPSFI - Scanning Frequency Information

    #* $GPTRF - Transit Fix Data
    #* $GPTTM - Tracked Target Message
    #* $GPVBW - Dual Ground/Water Speed
    #* $GPVDR - Set and Drift
    #* $GPVHW - Water Speed and Heading
    #* $GPVLW - Distance Traveled through the Water
    #* $GPVPW - Speed, Measured Parallel to Wind
    #* $GPVTG - Track Made Good and Ground Speed
    #* $GPWCV - Waypoint Closure Velocity
    #* $GPWNC - Distance, Waypoint to Waypoint
    #* $GPWPL - Waypoint Location
    #* $GPXDR - Transducer Measurements
    #* $GPXTE - Cross-Track Error, Measured
    #* $GPXTR - Cross-Track Error, Dead Reckoning

    #* $GPZFO - UTC & Time from Origin Waypoint
    #* $GPZTG - UTC & Time to Destination Waypoint
