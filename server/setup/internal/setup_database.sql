/*
 * Postgresql: Setup Auvsi Imaging database
 * NOTE: this script assumes you're starting from
 *       a fresh install of postgres, and may cause
 *       cause errors other wise
 */

 
/* Step 0: Destroy the current database and user if needed */
/* This will generate an error or two the first time you run it 
    since there is no database or user to delete */
DROP DATABASE auvsi;
DROP OWNED BY imaging_server;
REVOKE ALL PRIVILEGES ON ALL TABLES IN SCHEMA public FROM imaging_server;
REVOKE ALL PRIVILEGES ON ALL SEQUENCES IN SCHEMA public FROM imaging_server;
REVOKE ALL PRIVILEGES ON ALL FUNCTIONS IN SCHEMA public FROM imaging_server;
DROP USER imaging_server;

/* Step 1: create user `imaging_server` and database `auvsi` */
CREATE USER imaging_server WITH ENCRYPTED PASSWORD 'Byuauvs1';
CREATE DATABASE auvsi;-- WITH OWNER imaging_server;
GRANT CONNECT ON DATABASE auvsi TO imaging_server;
GRANT USAGE ON SCHEMA public TO imaging_server;

-- connect to the new database
\c auvsi;

/* Step 2: create all the tables */

CREATE TABLE "public"."incoming_image" (
  image_id serial NOT NULL,
  time_stamp timestamp NOT NULL,
  focal_length real, 
  image_path text NOT NULL,
  manual_tap boolean NOT NULL default FALSE,
  autonomous_tap boolean NOT NULL default FALSE,
  PRIMARY KEY ("image_id")
);

CREATE TABLE "public"."incoming_gps" (
  id serial NOT NULL,
  time_stamp timestamp NOT NULL,
  latitude real NOT NULL,
  longitude real NOT NULL,
  altitude real NOT NULL,
  PRIMARY KEY ("id")
);

CREATE TABLE "public"."incoming_state" (
  id serial NOT NULL,
  time_stamp timestamp NOT NULL,
  roll real NOT NULL,
  pitch real NOT NULL,
  yaw real NOT NULL,
  PRIMARY KEY ("id")
);

CREATE TABLE "public"."manual_cropped" (
  id serial NOT NULL,
  image_id int NOT NULL,
  time_stamp timestamp NOT NULL,
  cropped_path text NOT NULL,
  crop_coordinate_tl point,
  crop_coordinate_br point,
  tapped boolean NOT NULL default FALSE,
  PRIMARY KEY ("id")
);

-- These enums ensure that classification values are only set to valid AUVSI values
CREATE TYPE submit_status AS ENUM ('unsubmitted', 'inherited_submission', 'submitted');
CREATE TYPE color_t AS ENUM ('white', 'black', 'gray', 'red', 'blue', 'green', 'yellow', 'purple', 'brown', 'orange');
CREATE TYPE shape_t AS ENUM ('circle', 'semicircle', 'quarter_circle', 'triangle', 'square', 'rectangle', 'trapezoid', 'pentagon', 'hexagon', 'heptagon', 'octagon', 'star', 'cross');
CREATE TYPE orientation_t AS ENUM ('N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW');
CREATE TYPE target_type_t AS ENUM ('standard', 'off_axis', 'emergent');

CREATE TABLE "public"."outgoing_manual" (
  id serial NOT NULL,
  crop_id int NOT NULL,
  target int,
  type target_type_t,
  latitude real,
  longitude real,
  orientation orientation_t,
  shape shape_t,
  background_color color_t,
  alphanumeric text,
  alphanumeric_color color_t,
  description text default '',
  submitted submit_status NOT NULL default 'unsubmitted',
  PRIMARY KEY ("id"),
  UNIQUE ("crop_id")
);

CREATE TABLE "public"."outgoing_autonomous" (
  id serial NOT NULL,
  image_id int NOT NULL,
  crop_path text NOT NULL,
  target int, 
  type target_type_t,
  latitude real,
  longitude real,
  orientation orientation_t,
  shape shape_t,
  background_color color_t,
  alphanumeric text,
  alphanumeric_color color_t,
  description text default '',
  submitted submit_status NOT NULL default 'unsubmitted',
  PRIMARY KEY ("id")
);

CREATE TABLE "public"."submitted_target" (
  target_id int NOT NULL,
  autonomous boolean NOT NULL,
  crop_path text NOT NULL,
  PRIMARY KEY("target_id", "autonomous")
);

ALTER DATABASE auvsi OWNER TO imaging_server;
ALTER TABLE incoming_image OWNER TO imaging_server;
ALTER TABLE incoming_gps OWNER TO imaging_server;
ALTER TABLE incoming_state OWNER TO imaging_server;
ALTER TABLE manual_cropped OWNER TO imaging_server;
ALTER TABLE outgoing_autonomous OWNER TO imaging_server;
ALTER TABLE outgoing_manual OWNER TO imaging_server;
GRANT SELECT, INSERT, UPDATE, DELETE ON ALL TABLES IN SCHEMA public TO imaging_server;
GRANT USAGE, SELECT ON ALL SEQUENCES IN SCHEMA public TO imaging_server;
ALTER DEFAULT PRIVILEGES FOR USER imaging_server IN SCHEMA public GRANT SELECT, INSERT, UPDATE, DELETE ON TABLES TO imaging_server;


