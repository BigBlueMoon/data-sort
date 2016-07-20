'''
Program reads  data streaming live from sensors attached to the Arduino uno board
each line is delimited by (", ") and identified by a letter (A, B, C)
data is then stored in the appropriate table in the dataBase
'''

import serial
from serial import SerialException
import time
import datetime
import sys
import sqlite3

error_log = ""

#  ------------Function returns system time stamp------------------------


def get_time():
    # return a time stamp
    time_ = time.time()
    clock = datetime.datetime.fromtimestamp(time_).strftime('%Y-%m-%d %H:%M:%S')
    return clock


#  ------------------------Add Error log-------------------------------------
# adds error messages to the error log. Accepts a string argument
def add_error(message):
    global error_log
    try:
        #  opens the log file
        error_log = open("error-log.txt", 'a')  # 'a' = append to file & 'w' = overwrite file
        error_log.write(get_time() + " | " + message)
    except IOError:
        ioe = "couldn't open 'error log' file"
        print(ioe)


# ---------------------Connect to database database-------------------------------
try:
    # use this to commit, rollback and close connection
    con = sqlite3.connect('sensor-data-tables.db')

except SQLITE_CANTOPEN:
    e = "can't open file 'sensor-data-tables'"
    print(e)
    add_error(e)

#  use this for DMLs (Data Manipulation Languages)
cursor = con.cursor()


#  -------------------Functions check if tables exists---------------------
#  Methods return true if table exists, False if there is no table


def check_table_a():
    try:
        con.execute("select * from 'single_output_sensors'")
        return True
    except:      # <---  no need to log this exception!
        return False


def check_table_b():
    try:
        con.execute("select * from 'Accelerometer'")
        return True
    except:
        return False


def check_table_c():
    try:
        con.execute("select * from 'Compass'")
        return True
    except:
        return False


def check_table_d():
    try:
        con.execute("select * from 'GPS'")
        return True
    except:
       return False

#  ----------------Functions for Creating tables---------------------------------


def create_table_a():
    try:
        con.execute("CREATE TABLE single_output_sensors (`Row_Num`	  INTEGER PRIMARY KEY, "
                    "                                    `Time_stamp` TEXT , "
                    "                                    `Sound`	  INTEGER,"
                    "                                    `Water`	  TEXT,"
                    "                                    `Vibration`  INTEGER,"
                    "                                    `UV_Index`	  INTEGER,"
                    "                                    `Digital_Light`	INTEGER,"
                    "                                    `Gas`	            INTEGER,"
                    "                                    `Dust`	            INTEGER,"
                    "                                    `Temp`	            INTEGER)")
        # print("Table A was created")

    except sqlite3.OperationalError as e:
        print("Table A Was Not Created")
        print("Error: %s" % str(e))
        add_error(e)
    except sqlite_error as err:
        print("Error: " % err)
        add_error(err)


def create_table_b():
    try:
        con.execute("CREATE TABLE `Accelerometer` (	`row_num`	INTEGER PRIMARY KEY,"
                    "	                            `time_stamp`	TEXT,"
                    "                               `X` 	INTEGER,"
                    "                               `Y` 	INTEGER,"
                    "                           	`Z` 	INTEGER,"
                    "                           	`AX`	INTEGER,"
                    "                               `AY`	INTEGER,"
                    "                               `AZ`	INTEGER)")
        # print("Table b was created")

    except sqlite3.OperationalError as eb:
        print("Table B Was Not Created")
        print("Error: %s" % str(eb))
        add_error(eb)


def create_table_c():
    try:
        con.execute("CREATE TABLE `Compass` (`Row_ID`	INTEGER PRIMARY KEY,"
                    "	                     `time_stamp`	TEXT,"
                    "                      	 `Direction`	TEXT,	"
                    "                        `Angle`	INTEGER)")
        # print("Table C was created")

    except sqlite3.OperationalError as ec:
        print("Table C Was Not Created")
        print("Error: %s" % str(ec))
        add_error(ec)


def create_table_d():
    try:
        con.execute("CREATE TABLE `GPS` (`row_id`	    INTEGER PRIMARY KEY,"
                    "                    `time stamp`	TEXT,"
                    "                  	 `latitude`	    INTEGER,"
                    "                    `longitude`	INTEGER)")
        # print("Table D was created")

    except sqlite3.OperationalError as ed:
        print("Table D Was Not Created")
        print("Error: %s" % str(ed))
        add_error(ed)


#  -----------------Calling Check tables/create tables functions-----------------------
# methods return true if the tables exist and false if not.

if check_table_a():
    print("table A exists")
else:
    try:
        create_table_a()
        print("Table A Created")

    except SQLITE_ERROR:
        con.rollback()
        e = "Error: table A (Single output sensors) was not created"
        print(e)
        add_error(e)

# -----------------------------

if check_table_b():
    print("table B exists")
else:
    try:
        create_table_b()
        print("Table B Created")

    except SQLITE_ERROR:
        con.rollback()
        e = "Error: table B (Accelerometer) was not created"
        print(e)
        add_error(e)

# ------------------------------

if check_table_c():
    print("table C exists")
else:
    try:
        create_table_c()
        print("Table C Created")

    except SQLITE_ERROR:
        con.rollback()
        e = "Error: table C (Compass) was not created"
        print(e)
        add_error(e)

# -----------------------------

if check_table_d():
    print("table D exists")
else:
    try:
        create_table_d()
        print("Table D Created")

    except SQLITE_ERROR:
        con.rollback()
        e = "Error: table D (GPS) was not created"
        print(e)
        add_error(e)


#  -------------------Functions Insert data into tables-------------------------------

# make exceptions more detailed so that the error messages more meaningful!!!
# Log all exceptions to a file (not table, outside of database file) with time stamp

def table_a(row_num, time_stamp, sound, water, vibration, uv_index, digital_light, gas, dust, temp):
    #  first table takes single output sensors
    try:
        cursor.execute("insert into 'single_output_sensors' "
                       "values(? , ? , ? , ?, ?, ? ,? ,? , ?, ? )",
                       (row_num, time_stamp, sound, water, vibration, uv_index, digital_light, gas, dust, temp))
        con.commit()
        print("%d rows were inserted into table A" % cursor.rowcount)  # counts the number of rows inserted

    except sqlite3.OperationalError as er:
        con.rollback()
        print("Operational Error: %s" % er)
        add_error(er)

    except sqlite3.IntegrityError as integ:
        con.rollback()
        print("Integrity Error: " % integ)
        add_error(integ)

    except sqlite3.DataError as Der:
        con.rollback()
        print("Data Error: " % Der)
        add_error(Der)

    except sqlite_full as full:
        con.rollback()
        print("Full Error: " % full)
        add_error(full)

# ---------------------------------------------


def table_b(row_num, time_stamp, x, y, z, ax, ay, az):
    #  table B takes the Accelerometer data
    try:
        cursor.execute("insert into 'Accelerometer' "
                       "Values(?, ?, ?, ?, ?, ?, ?, ?)",
                       (row_num, time_stamp, x, y, z, ax, ay, az))
        con.commit()
        print("%d rows were inserted into table B" % cursor.rowcount)  # counts the number of rows inserted

    except sqlite3.OperationalError as er:
        con.rollback()
        #  e = sys.exc_info()[0]
        print("Operational Error: %s" % er)
        add_error(er)

    except sqlite3.IntegrityError as integ:
        con.rollback()
        print("Integrity Error: " % integ)
        add_error(integ)

    except sqlite3.DataError as Der:
        con.rollback()
        print("Data Error: " % Der)
        add_error(Der)

    except sqlite_full as full:
        con.rollback()
        print("Full Error: " % full)
        add_error(full)

# ---------------------------------------------


def table_c(row_num, time_stamp, direction, angle):
    #  table C takes Compass data
    try:
        cursor.execute("insert into 'Compass' "
                       "Values(?, ?, ?, ?)",
                       (row_num, time_stamp,  direction, angle))
        con.commit()
        print("%d rows were inserted into table C" % cursor.rowcount)  # counts the number of rows inserted

    except sqlite3.OperationalError as er:
        con.rollback()
        print("Operational Error: %s" % er)
        add_error(er)

    except sqlite3.IntegrityError as integ:
        con.rollback()
        print("Integrity Error: " % integ)
        add_error(integ)

    except sqlite3.DataError as Der:
        con.rollback()
        print("Data Error: " % Der)
        add_error(Der)

    except sqlite_full as full:
        con.rollback()
        print("Full Error: " % full)
        add_error(full)

# ---------------------------------------------


def table_d(row_num, time_stamp, latitude, longitude):
    #  table D takes GPS data
    try:
        cursor.execute("insert into 'Compass' "
                       "Values(?, ?, ?, ?)",
                       (row_num, time_stamp,  latitude, longitude))
        con.commit()
        print("%d rows were inserted into table D" % cursor.rowcount)  # counts the number of rows inserted

    except sqlite3.OperationalError as er:
        con.rollback()
        print("Operational Error: %s" % er)
        add_error(er)

    except sqlite3.IntegrityError as integ:
        con.rollback()
        print("Integrity Error: " % integ)
        add_error(integ)

    except sqlite3.DataError as Der:
        con.rollback()
        print("Data Error: " % Der)
        add_error(Der)

    except sqlite_full as full:
        con.rollback()
        print("Full Error: " % full)
        add_error(full)

#  ----------Array list to hold incoming data after delimitation---------------

sensor_data = []

#  ------------Entry point of the program-------------------

'''
Big delays between different sensors cause extra white space or even new lines to be printed here
first three lines are garbage data

'''
try:
    # reads from port COM4 and assigns the data to SER
    with serial.Serial('COM4', 9600, timeout=1) as ser:

        ser.readline()
        ser.readline()
        ser.readline()

        while True:
            line = ser.readline()  # method reads line by line from variable SER

            # converts incoming bytes into string data
            lineStr = str(line, encoding='utf8')

            #  words delimited by coma
            sensor_data = str.split(lineStr.strip("\r\n"), ', ')

            # print(lineStr)

            if sensor_data[0] == "a":

                try:
                    # sound, water, vibration, uv_index, digital_light, gas, dust, temp
                    table_a(None, get_time(), sensor_data[1], sensor_data[2], sensor_data[3], sensor_data[4], sensor_data[5],
                            sensor_data[6], sensor_data[7], sensor_data[8])
                except IndexError:
                    e = "List Index out of range in row A"
                    print(e)
                    add_error(e)

            elif sensor_data[0] == "b":

                try:
                    # X, Y, Z, & Accelerated X, Y, Z
                    table_b(None, get_time(), sensor_data[1], sensor_data[2], sensor_data[3], sensor_data[4], sensor_data[5],
                            sensor_data[6])
                except IndexError:
                    e = "List Index out of range in row B"
                    print(e)
                    add_error(e)

            elif sensor_data[0] == "c":
                try:
                    # direction, angle
                    table_c(None, get_time(), sensor_data[1], sensor_data[2])
                except IndexError:
                    e = "List Index out of range in row C"
                    print(e)
                    add_error(e)

            elif sensor_data[0] == "d":
                try:
                    # latitude, longitude
                    table_d(None, get_time(), sensor_data[1], sensor_data[2])
                except IndexError:
                    e = "List Index Out of range in row D"

            else:
                er = "No row ID Found"
                print(er)
                add_error(er)

            #  Clears the List Array
            sensor_data = []

except SerialException as se:
    print(se)
    add_error(str(se))

except Exception as e:
    print(e)
    add_error(str(e))

# closes the database connection
con.close()
# closes the error lof file
error_log.close()


